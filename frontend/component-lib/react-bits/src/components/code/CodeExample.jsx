import { useMemo } from 'react';
import { getLanguage } from '../../utils/utils';
import { useComponentPropsContext } from '../../hooks/useComponentPropsContext';
import { generatePropsString } from '../../utils/codeGeneration';
import CliInstallation from './CliInstallation';
import CodeHighlighter from './CodeHighlighter';
import CodeOptions, { CSS, Tailwind, TSCSS, TSTailwind } from './CodeOptions';

const SKIP_KEYS = new Set(['tailwind', 'css', 'tsTailwind', 'tsCode', 'dependencies']);

/**
 * Injects current prop values into a usage code string.
 * Finds the component JSX and replaces its props with the current values.
 */
function injectPropsIntoCode(usageCode, props, defaultProps, componentName) {
  if (!usageCode || !props || !componentName) return usageCode;

  // Generate props string from current values (without base indentation - we'll add it later)
  const propsString = generatePropsString(props, defaultProps, {
    exclude: ['className', 'key', 'ref', 'children', 'onAnimationComplete', 'onLetterAnimationComplete'],
    indent: 2
  });

  if (!propsString) return usageCode;

  // Match the component tag and capture the leading whitespace for indentation
  // This regex captures: leading whitespace, opening tag with all attributes, and closing
  // Use (?![a-zA-Z]) negative lookahead to avoid matching components that start with the same name
  // (e.g., avoid matching ScrollStackItem when componentName is ScrollStack)
  const selfClosingRegex = new RegExp(`(^[ \\t]*)(<${componentName}(?![a-zA-Z]))([\\s\\S]*?)(\\/>)`, 'gm');
  const openingTagRegex = new RegExp(`(^[ \\t]*)(<${componentName}(?![a-zA-Z]))([\\s\\S]*?)(>)`, 'gm');

  let result = usageCode;
  let matched = false;

  // Handle self-closing tags: <Component ... />
  result = result.replace(selfClosingRegex, (match, indent, openTag, existingProps, closeTag) => {
    matched = true;
    // Extract className if present in existing props
    const classMatch = existingProps.match(/className="[^"]*"/);
    const classString = classMatch ? `\n${indent}  ${classMatch[0]}` : '';

    // Indent each prop line with the base indentation
    const indentedProps = propsString
      .split('\n')
      .map(line => indent + line)
      .join('\n');

    return `${indent}${openTag.trim()}\n${indentedProps}${classString}\n${indent}${closeTag}`;
  });

  if (matched) return result;

  // Handle opening tags: <Component ...>children</Component>
  result = result.replace(openingTagRegex, (match, indent, openTag, existingProps, closeTag) => {
    // Extract className if present in existing props
    const classMatch = existingProps.match(/className="[^"]*"/);
    const classString = classMatch ? `\n${indent}  ${classMatch[0]}` : '';

    // Indent each prop line with the base indentation
    const indentedProps = propsString
      .split('\n')
      .map(line => indent + line)
      .join('\n');

    return `${indent}${openTag.trim()}\n${indentedProps}${classString}\n${indent}${closeTag}`;
  });

  return result;
}

const CodeExample = ({ codeObject, componentName }) => {
  const { tailwind, css, tsTailwind, tsCode, code, usage, dependencies } = codeObject;
  const { props, defaultProps, hasChanges } = useComponentPropsContext();

  // Generate dynamic usage code with current props
  const dynamicUsage = useMemo(() => {
    if (!usage || !hasChanges || !componentName) return usage;
    return injectPropsIntoCode(usage, props, defaultProps, componentName);
  }, [usage, props, defaultProps, hasChanges, componentName]);

  const renderCssSection = () =>
    css && (
      <>
        <h2 className="demo-title">CSS</h2>
        <CodeHighlighter snippetId="css" language="css" codeString={css} />
      </>
    );

  return (
    <>
      <CliInstallation deps={dependencies} />

      {/* Dynamic Usage Section - shows first if props changed */}
      {dynamicUsage && (
        <div>
          <h2 className="demo-title">
            Usage {hasChanges && <span style={{ color: '#B19EEF', fontSize: '12px' }}>(with your settings)</span>}
          </h2>
          <CodeHighlighter snippetId="usage" language="jsx" codeString={dynamicUsage} />
        </div>
      )}

      {Object.entries(codeObject).map(([name, snippet]) => {
        if (SKIP_KEYS.has(name)) return null;
        if (name === 'usage') return null; // Already rendered above

        if (name === 'code' || name === 'tsCode') {
          return (
            <div key={name}>
              <h2 className="demo-title">{name}</h2>
              <CodeOptions>
                {tailwind && (
                  <Tailwind>
                    <CodeHighlighter snippetId="code" language="jsx" codeString={tailwind} />
                  </Tailwind>
                )}
                {code && (
                  <CSS>
                    <CodeHighlighter snippetId="code" language="jsx" codeString={code} />
                    {css && renderCssSection()}
                  </CSS>
                )}
                {tsTailwind && (
                  <TSTailwind>
                    <CodeHighlighter snippetId="code" language="tsx" codeString={tsTailwind} />
                  </TSTailwind>
                )}
                {tsCode && (
                  <TSCSS>
                    <CodeHighlighter snippetId="code" language="tsx" codeString={tsCode} />
                    {renderCssSection()}
                  </TSCSS>
                )}
              </CodeOptions>
            </div>
          );
        }

        return (
          <div key={name}>
            <h2 className="demo-title">{name}</h2>
            <CodeHighlighter snippetId={name} language={getLanguage(name)} codeString={snippet} />
          </div>
        );
      })}
    </>
  );
};

export default CodeExample;
