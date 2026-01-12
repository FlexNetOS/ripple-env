#!/usr/bin/env python3
"""
AI Service Load Testing

Tests the performance and reliability of AI services (LocalAI, AGiXT)
under various load conditions.

Usage:
    python ai_load_test.py [--endpoint URL] [--requests N] [--concurrency N]
"""

import argparse
import asyncio
import json
import statistics
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import aiohttp
except ImportError:
    print("aiohttp not installed. Run: pip install aiohttp")
    sys.exit(1)


@dataclass
class LoadTestResult:
    """Results from a load test run."""

    total_requests: int = 0
    successful_requests: int = 0
    failed_requests: int = 0
    response_times: list = field(default_factory=list)
    errors: list = field(default_factory=list)
    start_time: float = 0
    end_time: float = 0

    @property
    def duration(self) -> float:
        return self.end_time - self.start_time

    @property
    def requests_per_second(self) -> float:
        if self.duration > 0:
            return self.total_requests / self.duration
        return 0

    @property
    def success_rate(self) -> float:
        if self.total_requests > 0:
            return (self.successful_requests / self.total_requests) * 100
        return 0

    @property
    def avg_response_time(self) -> float:
        if self.response_times:
            return statistics.mean(self.response_times)
        return 0

    @property
    def p50_response_time(self) -> float:
        if self.response_times:
            return statistics.median(self.response_times)
        return 0

    @property
    def p95_response_time(self) -> float:
        if len(self.response_times) >= 20:
            sorted_times = sorted(self.response_times)
            idx = int(len(sorted_times) * 0.95)
            return sorted_times[idx]
        return max(self.response_times) if self.response_times else 0

    @property
    def p99_response_time(self) -> float:
        if len(self.response_times) >= 100:
            sorted_times = sorted(self.response_times)
            idx = int(len(sorted_times) * 0.99)
            return sorted_times[idx]
        return max(self.response_times) if self.response_times else 0


class AILoadTester:
    """Load tester for AI services."""

    def __init__(
        self,
        base_url: str = "http://localhost:8080",
        timeout: int = 30,
    ):
        self.base_url = base_url.rstrip("/")
        self.timeout = aiohttp.ClientTimeout(total=timeout)

    async def check_health(self) -> bool:
        """Check if the service is healthy."""
        try:
            async with aiohttp.ClientSession(timeout=self.timeout) as session:
                async with session.get(f"{self.base_url}/readyz") as resp:
                    return resp.status == 200
        except Exception:
            return False

    async def _make_request(
        self,
        session: aiohttp.ClientSession,
        endpoint: str,
        method: str = "GET",
        data: Optional[dict] = None,
    ) -> tuple[bool, float, Optional[str]]:
        """Make a single request and return (success, response_time, error)."""
        start = time.perf_counter()
        try:
            if method == "GET":
                async with session.get(f"{self.base_url}{endpoint}") as resp:
                    await resp.text()
                    elapsed = time.perf_counter() - start
                    return resp.status < 400, elapsed, None
            else:
                async with session.post(
                    f"{self.base_url}{endpoint}",
                    json=data,
                    headers={"Content-Type": "application/json"},
                ) as resp:
                    await resp.text()
                    elapsed = time.perf_counter() - start
                    return resp.status < 400, elapsed, None
        except asyncio.TimeoutError:
            elapsed = time.perf_counter() - start
            return False, elapsed, "Timeout"
        except Exception as e:
            elapsed = time.perf_counter() - start
            return False, elapsed, str(e)

    async def run_load_test(
        self,
        endpoint: str = "/v1/models",
        method: str = "GET",
        data: Optional[dict] = None,
        num_requests: int = 100,
        concurrency: int = 10,
    ) -> LoadTestResult:
        """Run a load test against the specified endpoint."""
        result = LoadTestResult()
        result.start_time = time.perf_counter()

        semaphore = asyncio.Semaphore(concurrency)

        async def bounded_request(session: aiohttp.ClientSession) -> None:
            async with semaphore:
                success, response_time, error = await self._make_request(
                    session, endpoint, method, data
                )
                result.total_requests += 1
                result.response_times.append(response_time)
                if success:
                    result.successful_requests += 1
                else:
                    result.failed_requests += 1
                    if error:
                        result.errors.append(error)

        async with aiohttp.ClientSession(timeout=self.timeout) as session:
            tasks = [bounded_request(session) for _ in range(num_requests)]
            await asyncio.gather(*tasks)

        result.end_time = time.perf_counter()
        return result

    async def test_models_endpoint(
        self, num_requests: int = 50, concurrency: int = 5
    ) -> LoadTestResult:
        """Test the /v1/models endpoint."""
        return await self.run_load_test(
            endpoint="/v1/models",
            method="GET",
            num_requests=num_requests,
            concurrency=concurrency,
        )

    async def test_health_endpoint(
        self, num_requests: int = 100, concurrency: int = 10
    ) -> LoadTestResult:
        """Test the health check endpoint."""
        return await self.run_load_test(
            endpoint="/readyz",
            method="GET",
            num_requests=num_requests,
            concurrency=concurrency,
        )

    async def test_chat_completion(
        self, num_requests: int = 10, concurrency: int = 2
    ) -> LoadTestResult:
        """Test chat completion endpoint (if a model is loaded)."""
        data = {
            "model": "test",
            "messages": [{"role": "user", "content": "Hello"}],
            "max_tokens": 10,
        }
        return await self.run_load_test(
            endpoint="/v1/chat/completions",
            method="POST",
            data=data,
            num_requests=num_requests,
            concurrency=concurrency,
        )


def print_results(name: str, result: LoadTestResult) -> None:
    """Print formatted load test results."""
    print(f"\n{'=' * 60}")
    print(f"  {name}")
    print(f"{'=' * 60}")
    print(f"  Total Requests:     {result.total_requests}")
    print(f"  Successful:         {result.successful_requests}")
    print(f"  Failed:             {result.failed_requests}")
    print(f"  Success Rate:       {result.success_rate:.1f}%")
    print(f"  Duration:           {result.duration:.2f}s")
    print(f"  Requests/sec:       {result.requests_per_second:.2f}")
    print(f"  Avg Response Time:  {result.avg_response_time * 1000:.2f}ms")
    print(f"  P50 Response Time:  {result.p50_response_time * 1000:.2f}ms")
    print(f"  P95 Response Time:  {result.p95_response_time * 1000:.2f}ms")
    print(f"  P99 Response Time:  {result.p99_response_time * 1000:.2f}ms")

    if result.errors:
        unique_errors = list(set(result.errors[:5]))
        print(f"  Sample Errors:      {unique_errors}")


def export_results(results: dict[str, LoadTestResult], filename: str) -> None:
    """Export results to JSON file."""
    data = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "tests": {},
    }

    for name, result in results.items():
        data["tests"][name] = {
            "total_requests": result.total_requests,
            "successful_requests": result.successful_requests,
            "failed_requests": result.failed_requests,
            "success_rate": result.success_rate,
            "duration": result.duration,
            "requests_per_second": result.requests_per_second,
            "avg_response_time_ms": result.avg_response_time * 1000,
            "p50_response_time_ms": result.p50_response_time * 1000,
            "p95_response_time_ms": result.p95_response_time * 1000,
            "p99_response_time_ms": result.p99_response_time * 1000,
        }

    with open(filename, "w") as f:
        json.dump(data, f, indent=2)

    print(f"\nResults exported to: {filename}")


async def main() -> int:
    parser = argparse.ArgumentParser(description="AI Service Load Testing")
    parser.add_argument(
        "--endpoint",
        default="http://localhost:8080",
        help="Base URL of the AI service",
    )
    parser.add_argument(
        "--requests",
        type=int,
        default=50,
        help="Number of requests per test",
    )
    parser.add_argument(
        "--concurrency",
        type=int,
        default=5,
        help="Number of concurrent requests",
    )
    parser.add_argument(
        "--output",
        default="load-test-results.json",
        help="Output file for results",
    )
    args = parser.parse_args()

    print(f"AI Service Load Testing")
    print(f"Target: {args.endpoint}")
    print(f"Requests: {args.requests}, Concurrency: {args.concurrency}")

    tester = AILoadTester(base_url=args.endpoint)

    # Check health first
    print("\nChecking service health...")
    if not await tester.check_health():
        print("WARNING: Service health check failed. Tests may fail.")

    results: dict[str, LoadTestResult] = {}

    # Run health endpoint test
    print("\nRunning health endpoint load test...")
    results["health"] = await tester.test_health_endpoint(
        num_requests=args.requests * 2,
        concurrency=args.concurrency * 2,
    )
    print_results("Health Endpoint (/readyz)", results["health"])

    # Run models endpoint test
    print("\nRunning models endpoint load test...")
    results["models"] = await tester.test_models_endpoint(
        num_requests=args.requests,
        concurrency=args.concurrency,
    )
    print_results("Models Endpoint (/v1/models)", results["models"])

    # Run chat completion test (lower volume due to compute cost)
    print("\nRunning chat completion load test...")
    results["chat"] = await tester.test_chat_completion(
        num_requests=min(10, args.requests // 5),
        concurrency=min(2, args.concurrency),
    )
    print_results("Chat Completion (/v1/chat/completions)", results["chat"])

    # Export results
    export_results(results, args.output)

    # Summary
    print(f"\n{'=' * 60}")
    print("  SUMMARY")
    print(f"{'=' * 60}")

    all_passed = True
    for name, result in results.items():
        status = "✅ PASS" if result.success_rate >= 95 else "❌ FAIL"
        if result.success_rate < 95:
            all_passed = False
        print(f"  {name}: {status} ({result.success_rate:.1f}% success)")

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
