import React from 'react';
import CodeBlock from '@theme-original/CodeBlock';
import { useCurrentLocale, isRTL } from '@site/src/utils/locale';

const CustomCodeBlock = (props) => {
  const currentLocale = useCurrentLocale();
  const isRTLLayout = isRTL(currentLocale);

  // Code blocks should remain LTR even in RTL layout
  return (
    <div dir="ltr" className="code-block-container">
      <CodeBlock {...props} />
    </div>
  );
};

export default CustomCodeBlock;