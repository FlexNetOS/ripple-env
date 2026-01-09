# ARIA Configuration Consistency Policies
# Use with: conftest test --policy .claude/policies/

package main

# Deny references to files at root that should be in docs/
deny[msg] {
    input.references[_].path == path
    not startswith(path, "docs/")
    endswith(path, "_SPEC.md")
    msg := sprintf("SPEC files should be in docs/: %s", [path])
}

deny[msg] {
    input.references[_].path == path
    not startswith(path, "docs/")
    endswith(path, "_REPORT.md")
    msg := sprintf("REPORT files should be in docs/reports/: %s", [path])
}

# Deny references to README.md in skills (should be SKILL.md)
deny[msg] {
    input.references[_].path == path
    contains(path, "skills/")
    endswith(path, "/README.md")
    msg := sprintf("Skill files should be SKILL.md, not README.md: %s", [path])
}

# Warn on docker-compose files not in docker/
warn[msg] {
    input.references[_].path == path
    startswith(path, "docker-compose.")
    not startswith(path, "docker/")
    msg := sprintf("Docker compose files should be in docker/: %s", [path])
}

# Deny broken relative paths
deny[msg] {
    input.references[_].path == path
    input.references[_].exists == false
    msg := sprintf("Broken reference to non-existent file: %s", [path])
}

# Warn on inconsistent path case
warn[msg] {
    paths := [p | input.references[_].path == p]
    path1 := paths[i]
    path2 := paths[j]
    i < j
    lower(path1) == lower(path2)
    path1 != path2
    msg := sprintf("Inconsistent path case: '%s' vs '%s'", [path1, path2])
}

# Deny 13-layer references (should be 14 domains)
deny[msg] {
    input.content
    contains(input.content, "13-layer")
    not contains(input.content, "13 layers + ")
    msg := "Use '14 domains' or '13 layers + security cross-cutting', not '13-layer'"
}

# Ensure env files are gitignored
deny[msg] {
    input.file == ".gitignore"
    not contains(input.content, ".claude/config/env")
    msg := ".claude/config/env should be in .gitignore"
}
