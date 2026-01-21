#!/usr/bin/env bash
# Validate GitHub Copilot custom agent configuration files

set -euo pipefail

AGENTS_DIR=".github/agents"
ERRORS=0

echo "🔍 Validating GitHub Copilot custom agents..."
echo ""

if [ ! -d "$AGENTS_DIR" ]; then
    echo "❌ Error: $AGENTS_DIR directory not found"
    exit 1
fi

# Find all .agent.md files and iterate safely
if ! find "$AGENTS_DIR" -name "*.agent.md" -type f -print0 | grep -qz .; then
    echo "⚠️  Warning: No .agent.md files found in $AGENTS_DIR"
    exit 0
fi

while IFS= read -r -d '' agent_file; do
    echo "📝 Validating: $agent_file"
    
    # Check if file has YAML frontmatter
    if ! head -n 1 "$agent_file" | grep -q "^---$"; then
        echo "   ❌ Missing YAML frontmatter opening (---)"
        ERRORS=$((ERRORS + 1))
        continue
    fi
    
    # Extract frontmatter (between first and second ---)
    frontmatter=$(sed -n '/^---$/,/^---$/p' "$agent_file" | sed '1d;$d')
    
    if [ -z "$frontmatter" ]; then
        echo "   ❌ Empty or invalid YAML frontmatter"
        ERRORS=$((ERRORS + 1))
        continue
    fi
    
    # Check required fields
    if ! echo "$frontmatter" | grep -q "^name:"; then
        echo "   ❌ Missing 'name' field in frontmatter"
        ERRORS=$((ERRORS + 1))
    else
        name=$(echo "$frontmatter" | grep "^name:" | sed 's/name: *//')
        echo "   ✅ Name: $name"
    fi
    
    if ! echo "$frontmatter" | grep -q "^description:"; then
        echo "   ❌ Missing 'description' field in frontmatter"
        ERRORS=$((ERRORS + 1))
    else
        description=$(echo "$frontmatter" | grep "^description:" | sed 's/description: *//')
        echo "   ✅ Description: $description"
    fi
    
    # Check optional but recommended fields
    if echo "$frontmatter" | grep -q "^tools:"; then
        tools=$(echo "$frontmatter" | grep "^tools:" | sed 's/tools: *//')
        echo "   ✅ Tools: $tools"
    else
        echo "   ⚠️  Optional 'tools' field not specified"
    fi
    
    if echo "$frontmatter" | grep -q "^model:"; then
        model=$(echo "$frontmatter" | grep "^model:" | sed 's/model: *//')
        echo "   ✅ Model: $model"
    else
        echo "   ⚠️  Optional 'model' field not specified (will use default)"
    fi
    
    # Check if there's content after frontmatter
    # Extract content after the second --- delimiter
    body_lines=$(awk '/^---$/{count++; next} count==2{print}' "$agent_file" | wc -l)
    
    if [ "$body_lines" -lt 10 ]; then
        echo "   ⚠️  Agent body is very short ($body_lines lines)"
    else
        echo "   ✅ Agent body: $body_lines lines"
    fi
    
    echo ""
done < <(find "$AGENTS_DIR" -name "*.agent.md" -type f -print0)

# Summary
if [ $ERRORS -eq 0 ]; then
    echo "✅ All agent files validated successfully!"
    exit 0
else
    echo "❌ Validation failed with $ERRORS error(s)"
    exit 1
fi
