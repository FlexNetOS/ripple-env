import React, { useEffect, useMemo } from 'react';
import { CodeTab, PreviewTab, TabsLayout } from '../../components/common/TabsLayout';
import { Box } from '@chakra-ui/react';

import useComponentProps from '../../hooks/useComponentProps';
import { ComponentPropsProvider } from '../../components/context/ComponentPropsContext';
import Customize from '../../components/common/Preview/Customize';
import PreviewSlider from '../../components/common/Preview/PreviewSlider';
import logo from '../../assets/logos/react-bits-logo-small-black.svg';
import CodeExample from '../../components/code/CodeExample';
import useForceRerender from '../../hooks/useForceRerender';

import PropTable from '../../components/common/Preview/PropTable';

import MetallicPaint, { parseLogoImage } from '../../content/Animations/MetallicPaint/MetallicPaint';
import { metallicPaint } from '../../constants/code/Animations/metallicPaintCode';

const DEFAULT_PROPS = {
  edge: 0,
  patternScale: 2,
  refraction: 0.015,
  patternBlur: 0.005,
  liquid: 0.07,
  speed: 0.3
};

const LiquidPaperDemo = () => {
  const { props, updateProp, resetProps, hasChanges } = useComponentProps(DEFAULT_PROPS);
  const { edge, patternScale, refraction, patternBlur, liquid, speed } = props;

  const [imageData, setImageData] = React.useState(null);
  const [key, forceRerender] = useForceRerender();

  useEffect(() => {
    async function loadDefaultImage() {
      try {
        const response = await fetch(logo);
        const blob = await response.blob();
        const file = new File([blob], 'default.png', { type: blob.type });
        const { imageData } = await parseLogoImage(file);
        setImageData(imageData);
      } catch (err) {
        console.error('Error loading default image:', err);
      }
    }
    loadDefaultImage();
  }, []);

  const propData = useMemo(
    () => [
      {
        name: 'imageData',
        type: 'ImageData',
        default: 'none (required)',
        description:
          'The processed image data generated from parseLogoImage. This image data is used by the shader to create the liquid paper effect.'
      },
      {
        name: 'params',
        type: 'ShaderParams',
        default: '',
        description:
          'An object to configure the shader effect. Properties include: patternScale, refraction, edge, patternBlur, liquid, speed'
      }
    ],
    []
  );

  return (
    <ComponentPropsProvider props={props} defaultProps={DEFAULT_PROPS} resetProps={resetProps} hasChanges={hasChanges}>
      <TabsLayout>
        <PreviewTab>
          <Box position="relative" className="demo-container" h={400} overflow="hidden">
            <MetallicPaint
              key={key}
              imageData={imageData}
              params={{ edge, patternBlur, patternScale, refraction, speed, liquid }}
            />
          </Box>

          <Customize>
            <PreviewSlider
              title="Edge"
              min={0}
              max={2}
              step={0.1}
              value={edge}
              onChange={val => {
                updateProp('edge', val);
                forceRerender();
              }}
            />

            <PreviewSlider
              title="Pattern Scale"
              min={1}
              max={5}
              step={0.1}
              value={patternScale}
              onChange={val => {
                updateProp('patternScale', val);
                forceRerender();
              }}
            />

            <PreviewSlider
              title="Pattern Blur"
              min={0}
              max={0.1}
              step={0.001}
              value={patternBlur}
              onChange={val => {
                updateProp('patternBlur', val);
                forceRerender();
              }}
            />

            <PreviewSlider
              title="Refraction"
              min={0}
              max={0.1}
              step={0.01}
              value={refraction}
              onChange={val => {
                updateProp('refraction', val);
                forceRerender();
              }}
            />

            <PreviewSlider
              title="Liquid"
              min={0}
              max={1}
              step={0.01}
              value={liquid}
              onChange={val => {
                updateProp('liquid', val);
                forceRerender();
              }}
            />

            <PreviewSlider
              title="Speed"
              min={0}
              max={1}
              step={0.01}
              value={speed}
              onChange={val => {
                updateProp('speed', val);
                forceRerender();
              }}
            />
          </Customize>

          <PropTable data={propData} />
        </PreviewTab>

        <CodeTab>
          <CodeExample codeObject={metallicPaint} componentName="MetallicPaint" />
        </CodeTab>
      </TabsLayout>
    </ComponentPropsProvider>
  );
};

export default LiquidPaperDemo;
