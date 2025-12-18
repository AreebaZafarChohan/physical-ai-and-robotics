import React from 'react';
import OriginalDocPage from '@theme-original/DocPage';
import CTAFooterSection from '../../components/sections/CTAFooterSection';

type DocPageProps = {
  content: any;
  [key: string]: any;
};

export default function DocPageWrapper(props: DocPageProps) {
  return (
    <>
      <OriginalDocPage {...props} />
      <CTAFooterSection />
    </>
  );
}