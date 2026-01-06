import React from 'react';
import Navbar from '@theme-original/Navbar';
import { useCurrentLocale, isRTL } from '@site/src/utils/locale';

const CustomNavbar = (props) => {
  const currentLocale = useCurrentLocale();
  const isRTLLayout = isRTL(currentLocale);

  return (
    <div dir={isRTLLayout ? 'rtl' : 'ltr'} className={isRTLLayout ? 'rtl-layout' : 'ltr-layout'}>
      <Navbar {...props} />
    </div>
  );
};

export default CustomNavbar;