import { createContext, useMemo } from 'react';

/**
 * Context for sharing component demo props between PreviewTab and CodeTab.
 * This enables the Code tab to display usage examples with the exact
 * prop values the user has configured in the demo.
 */
const ComponentPropsContext = createContext({
  props: {},
  defaultProps: {},
  hasChanges: false,
  resetProps: () => {}
});

/**
 * Provider component for sharing demo props with CodeExample.
 *
 * @example
 * <ComponentPropsProvider
 *   props={props}
 *   defaultProps={DEFAULT_PROPS}
 *   resetProps={resetProps}
 *   hasChanges={hasChanges}
 * >
 *   <TabsLayout>...</TabsLayout>
 * </ComponentPropsProvider>
 */
export function ComponentPropsProvider({ children, props, defaultProps, resetProps, hasChanges }) {
  const value = useMemo(
    () => ({
      props,
      defaultProps,
      hasChanges,
      resetProps
    }),
    [props, defaultProps, hasChanges, resetProps]
  );

  return <ComponentPropsContext.Provider value={value}>{children}</ComponentPropsContext.Provider>;
}

export default ComponentPropsContext;
