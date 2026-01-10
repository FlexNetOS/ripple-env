#!/usr/bin/env bash
# Health check script with retry logic for CI workflows
#
# Usage: ./health-check.sh <URL> [MAX_RETRIES] [RETRY_DELAY] [SERVICE_NAME]
#
# Arguments:
#   URL          - The health check endpoint URL (required)
#   MAX_RETRIES  - Maximum number of retry attempts (default: 5)
#   RETRY_DELAY  - Initial delay between retries in seconds (default: 10)
#   SERVICE_NAME - Name of the service for logging (default: extracted from URL)
#
# Example:
#   ./health-check.sh http://localhost:8080/readyz 5 10 "LocalAI"

set -euo pipefail

URL="${1:-}"
MAX_RETRIES="${2:-5}"
RETRY_DELAY="${3:-10}"
SERVICE_NAME="${4:-}"

if [[ -z "$URL" ]]; then
    echo "Usage: $0 <URL> [MAX_RETRIES] [RETRY_DELAY] [SERVICE_NAME]"
    exit 1
fi

# Extract service name from URL if not provided
if [[ -z "$SERVICE_NAME" ]]; then
    SERVICE_NAME=$(echo "$URL" | sed -E 's|https?://([^/:]+).*|\1|')
fi

echo "Health check for $SERVICE_NAME ($URL)"
echo "  Max retries: $MAX_RETRIES, Initial delay: ${RETRY_DELAY}s"

for i in $(seq 1 "$MAX_RETRIES"); do
    if curl -sf --connect-timeout 5 --max-time 10 "$URL" >/dev/null 2>&1; then
        echo "✅ $SERVICE_NAME is healthy (attempt $i/$MAX_RETRIES)"
        exit 0
    fi

    if [[ $i -eq $MAX_RETRIES ]]; then
        echo "❌ $SERVICE_NAME failed health check after $MAX_RETRIES attempts"
        exit 1
    fi

    # Exponential backoff with jitter
    CURRENT_DELAY=$((RETRY_DELAY * i))
    echo "⏳ Attempt $i/$MAX_RETRIES failed, retrying in ${CURRENT_DELAY}s..."
    sleep "$CURRENT_DELAY"
done

exit 1
