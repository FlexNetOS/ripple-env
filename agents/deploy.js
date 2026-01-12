#!/usr/bin/env node
/**
 * Agent Deployment Manager
 * P1-013: Deploy and manage AI agents in Kubernetes
 */

import { readFile } from 'fs/promises';
import { exec } from 'child_process';
import { promisify } from 'util';
import chalk from 'chalk';

const execAsync = promisify(exec);

const AGENTS = {
  'build-agent': {
    manifest: 'manifests/agents/build-agent.yaml',
    description: 'ROS2 build and compilation agent',
  },
  'test-agent': {
    manifest: 'manifests/agents/test-agent.yaml',
    description: 'Automated testing and validation agent',
  },
  'deploy-agent': {
    manifest: 'manifests/agents/deploy-agent.yaml',
    description: 'Deployment orchestration agent',
  },
  'monitor-agent': {
    manifest: 'manifests/agents/monitor-agent.yaml',
    description: 'Infrastructure monitoring agent',
  },
};

async function deployAgent(agentName) {
  const agent = AGENTS[agentName];
  if (!agent) {
    console.error(chalk.red(`Error: Unknown agent '${agentName}'`));
    console.log(chalk.yellow('Available agents:'));
    Object.keys(AGENTS).forEach(name => {
      console.log(`  - ${chalk.cyan(name)}: ${AGENTS[name].description}`);
    });
    process.exit(1);
  }

  try {
    console.log(chalk.blue(`Deploying ${agentName}...`));
    const { stdout, stderr } = await execAsync(`kubectl apply -f ${agent.manifest}`);
    if (stdout) console.log(stdout);
    if (stderr) console.error(chalk.yellow(stderr));
    console.log(chalk.green(`✓ ${agentName} deployed successfully`));
  } catch (error) {
    console.error(chalk.red(`✗ Failed to deploy ${agentName}:`), error.message);
    process.exit(1);
  }
}

async function deployAll() {
  console.log(chalk.blue('Deploying all agents...'));
  for (const agentName of Object.keys(AGENTS)) {
    await deployAgent(agentName);
  }
  console.log(chalk.green('\n✓ All agents deployed'));
}

async function listAgents() {
  console.log(chalk.cyan('\nAvailable Agents:\n'));
  Object.entries(AGENTS).forEach(([name, info]) => {
    console.log(`  ${chalk.bold(name)}`);
    console.log(`    Description: ${info.description}`);
    console.log(`    Manifest: ${info.manifest}\n`);
  });
}

async function checkStatus() {
  console.log(chalk.blue('Checking agent status...\n'));
  try {
    const { stdout } = await execAsync('kubectl get pods -n flexstack-agents -o wide');
    console.log(stdout);
  } catch (error) {
    console.error(chalk.red('Error checking status:'), error.message);
  }
}

// CLI
const command = process.argv[2];
const agentName = process.argv[3];

console.log(chalk.bold('\n🤖 Agent Deployment Manager\n'));

switch (command) {
  case 'deploy':
    if (agentName) {
      await deployAgent(agentName);
    } else {
      await deployAll();
    }
    break;
  case 'list':
    await listAgents();
    break;
  case 'status':
    await checkStatus();
    break;
  default:
    console.log('Usage:');
    console.log('  npm run agents:deploy deploy [agent-name]  - Deploy agent(s)');
    console.log('  npm run agents:deploy list                 - List available agents');
    console.log('  npm run agents:deploy status               - Check agent status');
    process.exit(1);
}
