#!/usr/bin/env node
/**
 * Agent Monitoring Dashboard
 * P1-014: Monitor agent health and performance
 */

import { exec } from 'child_process';
import { promisify } from 'util';
import chalk from 'chalk';

const execAsync = promisify(exec);

async function getAgentMetrics() {
  try {
    const { stdout } = await execAsync(
      'kubectl get pods -n flexstack-agents -o json'
    );
    const data = JSON.parse(stdout);
    return data.items || [];
  } catch (error) {
    console.error(chalk.red('Error fetching metrics:'), error.message);
    return [];
  }
}

async function getAgentLogs(podName, lines = 50) {
  try {
    const { stdout } = await execAsync(
      `kubectl logs -n flexstack-agents ${podName} --tail=${lines}`
    );
    return stdout;
  } catch (error) {
    console.error(chalk.red(`Error fetching logs for ${podName}:`), error.message);
    return '';
  }
}

async function displayDashboard() {
  console.clear();
  console.log(chalk.bold.cyan('\n📊 Agent Monitoring Dashboard\n'));
  console.log(chalk.gray('═'.repeat(80)));

  const agents = await getAgentMetrics();

  if (agents.length === 0) {
    console.log(chalk.yellow('\nNo agents found in flexstack-agents namespace'));
    console.log(chalk.gray('\nDeploy agents with: npm run agents:deploy deploy\n'));
    return;
  }

  console.log(chalk.bold('\nActive Agents:\n'));

  agents.forEach(agent => {
    const name = agent.metadata.name;
    const status = agent.status.phase;
    const ready = agent.status.containerStatuses?.[0]?.ready ? '✓' : '✗';
    const restarts = agent.status.containerStatuses?.[0]?.restartCount || 0;
    const age = calculateAge(agent.metadata.creationTimestamp);

    const statusColor = status === 'Running' ? chalk.green : chalk.yellow;
    const readyColor = ready === '✓' ? chalk.green : chalk.red;

    console.log(`  ${readyColor(ready)} ${chalk.bold(name)}`);
    console.log(`     Status: ${statusColor(status)}`);
    console.log(`     Restarts: ${restarts}`);
    console.log(`     Age: ${age}\n`);
  });

  console.log(chalk.gray('═'.repeat(80)));
  console.log(chalk.dim('\nPress Ctrl+C to exit\n'));
}

function calculateAge(timestamp) {
  const created = new Date(timestamp);
  const now = new Date();
  const diff = Math.floor((now - created) / 1000);

  if (diff < 60) return `${diff}s`;
  if (diff < 3600) return `${Math.floor(diff / 60)}m`;
  if (diff < 86400) return `${Math.floor(diff / 3600)}h`;
  return `${Math.floor(diff / 86400)}d`;
}

async function watchMode() {
  await displayDashboard();
  setInterval(async () => {
    await displayDashboard();
  }, 5000);
}

async function showLogs() {
  const agentName = process.argv[3];
  if (!agentName) {
    console.error(chalk.red('Error: Please specify an agent name'));
    console.log('Usage: npm run agents:monitor logs <agent-name>');
    process.exit(1);
  }

  console.log(chalk.cyan(`\n📋 Logs for ${agentName}:\n`));
  const logs = await getAgentLogs(agentName);
  console.log(logs);
}

// CLI
const command = process.argv[2];

console.log(chalk.bold('\n📊 Agent Monitoring Dashboard\n'));

switch (command) {
  case 'watch':
    await watchMode();
    break;
  case 'logs':
    await showLogs();
    break;
  default:
    await displayDashboard();
}
