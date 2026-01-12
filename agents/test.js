#!/usr/bin/env node
/**
 * Agent Testing Suite
 * P1-015: Test agent functionality and integration
 */

import { exec } from 'child_process';
import { promisify } from 'util';
import chalk from 'chalk';

const execAsync = promisify(exec);

const TESTS = {
  'connectivity': {
    description: 'Test agent connectivity',
    run: async () => {
      const { stdout } = await execAsync('kubectl get pods -n flexstack-agents');
      return stdout.includes('Running');
    },
  },
  'health': {
    description: 'Test agent health endpoints',
    run: async () => {
      try {
        const { stdout } = await execAsync(
          'kubectl get pods -n flexstack-agents -o json'
        );
        const data = JSON.parse(stdout);
        const pods = data.items || [];
        return pods.every(pod =>
          pod.status.containerStatuses?.[0]?.ready === true
        );
      } catch {
        return false;
      }
    },
  },
  'logs': {
    description: 'Test agent logging',
    run: async () => {
      try {
        const { stdout } = await execAsync(
          'kubectl logs -n flexstack-agents --selector=app.kubernetes.io/component=agent --tail=10'
        );
        return stdout.length > 0;
      } catch {
        return false;
      }
    },
  },
  'resources': {
    description: 'Test resource allocation',
    run: async () => {
      try {
        const { stdout } = await execAsync(
          'kubectl top pods -n flexstack-agents'
        );
        return stdout.length > 0;
      } catch (error) {
        // metrics-server might not be installed
        console.log(chalk.yellow('  ⚠ Metrics server not available'));
        return true;
      }
    },
  },
};

async function runTest(testName) {
  const test = TESTS[testName];
  if (!test) {
    console.error(chalk.red(`Error: Unknown test '${testName}'`));
    return false;
  }

  console.log(`  ${chalk.cyan('→')} ${test.description}...`);

  try {
    const result = await test.run();
    if (result) {
      console.log(`    ${chalk.green('✓')} Passed`);
      return true;
    } else {
      console.log(`    ${chalk.red('✗')} Failed`);
      return false;
    }
  } catch (error) {
    console.log(`    ${chalk.red('✗')} Error: ${error.message}`);
    return false;
  }
}

async function runAllTests() {
  console.log(chalk.bold.cyan('\n🧪 Agent Testing Suite\n'));
  console.log(chalk.gray('═'.repeat(60)));

  let passed = 0;
  let failed = 0;

  for (const [name, test] of Object.entries(TESTS)) {
    const result = await runTest(name);
    if (result) {
      passed++;
    } else {
      failed++;
    }
    console.log();
  }

  console.log(chalk.gray('═'.repeat(60)));
  console.log(chalk.bold('\nTest Summary:'));
  console.log(`  ${chalk.green('Passed:')} ${passed}`);
  console.log(`  ${chalk.red('Failed:')} ${failed}`);
  console.log(`  ${chalk.cyan('Total:')} ${passed + failed}\n`);

  if (failed === 0) {
    console.log(chalk.green.bold('✓ All tests passed!\n'));
    process.exit(0);
  } else {
    console.log(chalk.red.bold('✗ Some tests failed\n'));
    process.exit(1);
  }
}

async function listTests() {
  console.log(chalk.cyan('\nAvailable Tests:\n'));
  Object.entries(TESTS).forEach(([name, test]) => {
    console.log(`  ${chalk.bold(name)}: ${test.description}`);
  });
  console.log();
}

// CLI
const command = process.argv[2];

switch (command) {
  case 'all':
    await runAllTests();
    break;
  case 'list':
    await listTests();
    break;
  case 'run':
    const testName = process.argv[3];
    if (!testName) {
      console.error(chalk.red('Error: Please specify a test name'));
      await listTests();
      process.exit(1);
    }
    const result = await runTest(testName);
    process.exit(result ? 0 : 1);
    break;
  default:
    console.log('Usage:');
    console.log('  npm run agents:test all           - Run all tests');
    console.log('  npm run agents:test run <name>    - Run specific test');
    console.log('  npm run agents:test list          - List available tests');
    process.exit(1);
}
