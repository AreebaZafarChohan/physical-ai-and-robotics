import React from 'react';
import DocSidebar from '@theme-original/DocSidebar';
import { useCurrentLocale, isRTL } from '@site/src/utils/locale';

const CustomDocSidebar = (props) => {
  const currentLocale = useCurrentLocale();
  const isRTLLayout = isRTL(currentLocale);

  // Apply RTL-specific styling or modifications if needed
  const sidebarProps = {
    ...props,
    className: isRTLLayout ? `${props.className || ''} sidebar-rtl`.trim() : props.className
  };

  return (
    <div dir={isRTLLayout ? 'rtl' : 'ltr'} className={isRTLLayout ? 'rtl-layout' : 'ltr-layout'}>
      <DocSidebar {...sidebarProps} />
    </div>
  );
};

export default CustomDocSidebar;