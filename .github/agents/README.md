# GitHub Copilot Custom Agents

This directory contains custom GitHub Copilot agents that provide specialized assistance for this repository.

## Available Agents

### Security Auto-Fix Agent (`security-auto-fix.agent.md`)

**Purpose**: Automatically identifies and fixes security vulnerabilities detected by scanning tools.

**Capabilities**:
- Fixes code vulnerabilities (SQL injection, XSS, command injection, etc.)
- Updates vulnerable dependencies to patched versions
- Hardens container configurations
- Removes hard-coded secrets and implements secure alternatives
- Fixes configuration security issues

**How to Use**:

#### In GitHub Copilot Chat
```
@security_auto_fix Please analyze and fix the security issues in [file/directory]
```

#### After Security Scans
When security scanning tools (CodeQL, Trivy, Grype) detect vulnerabilities:
```
@security_auto_fix Fix the HIGH and CRITICAL vulnerabilities found in the latest security scan
```

#### For Specific Vulnerability Types
```
@security_auto_fix Fix the SQL injection vulnerability in src/database.py
@security_auto_fix Update the vulnerable requests package to a secure version
@security_auto_fix Remove hard-coded secrets from config files
```

**Priority Handling**:
- **P0 (Critical)**: CVSS >= 9.0 - Fixed immediately
- **P1 (High)**: CVSS >= 7.0 - Fixed with priority
- **P2 (Medium)**: CVSS >= 4.0 - Fixed if straightforward
- **P3 (Low)**: CVSS < 4.0 - Documented with recommendations

**Integration with CI/CD**:
The agent works with existing security workflows:
- `.github/workflows/security.yml` - Main security scanning
- `.github/workflows/container-security.yml` - Container image scanning

**Testing Fixes**:
After the agent applies fixes, always verify:
```bash
# Enter development environment
nix develop

# Run security scans
trivy fs --severity HIGH,CRITICAL .
gitleaks detect --source . --verbose

# Run tests
pixi run test
cargo test
nix flake check
```

## Creating New Custom Agents

To add a new custom agent:

1. Create a new `.agent.md` file in this directory
2. Add YAML frontmatter with agent configuration:
   ```yaml
   ---
   name: my_agent
   description: Brief description of what the agent does
   tools: [bash, edit, view, grep, glob]
   infer: true
   model: sonnet
   priority: normal
   ---
   ```
3. Write detailed instructions for the agent
4. Document the agent in this README

### Agent Configuration Options

- **name**: Unique identifier for the agent (use `snake_case`)
- **description**: Brief one-line description
- **tools**: List of tools the agent can use
  - `bash`: Execute shell commands
  - `edit`: Edit files
  - `view`: Read files
  - `grep`: Search file contents
  - `glob`: Find files by pattern
  - `create`: Create new files
  - `gh-advisory-database`: Check for known vulnerabilities
- **infer**: Allow agent to infer when to help (true/false)
- **model**: AI model to use (`sonnet`, `haiku`, `opus`)
- **priority**: Agent priority level (`critical`, `high`, `normal`, `low`)

## Best Practices

1. **Be Specific**: Give agents clear, specific instructions
2. **Review Changes**: Always review agent-generated changes before committing
3. **Test Thoroughly**: Run tests after agent applies fixes
4. **Document**: Keep this README updated with new agents and usage patterns
5. **Security First**: For security-related changes, verify fixes don't introduce new issues

## Related Documentation

- [GitHub Copilot Custom Agents Documentation](https://docs.github.com/en/copilot/how-tos/use-copilot-agents/coding-agent/create-custom-agents)
- [Custom Agents Configuration Reference](https://docs.github.com/en/copilot/reference/custom-agents-configuration)
- [Repository Security Documentation](../../SECURITY.md)
- [Security Workflow](../workflows/security.yml)

## Troubleshooting

**Agent Not Responding**:
- Ensure you're using the `@agent_name` syntax correctly
- Check that the agent file follows the correct YAML format
- Verify GitHub Copilot has access to your repository

**Agent Produces Incorrect Fixes**:
- Provide more context in your prompt
- Specify the exact file and line numbers
- Review and manually adjust the changes as needed

**Agent Cannot Fix Vulnerability**:
- Check if the issue requires architectural changes (may need human review)
- Verify if upstream patch is available
- Consider if fix would introduce breaking changes

## Support

For issues or questions:
1. Check existing GitHub Issues
2. Review security scanning logs in Actions
3. Consult the `.claude/agents/security-agent.md` for additional context
4. Create a new issue with `[agent]` prefix
