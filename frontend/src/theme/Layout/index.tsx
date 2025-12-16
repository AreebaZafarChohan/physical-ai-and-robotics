import React from 'react';
import Layout from '@theme-original/Layout';

type LayoutProps = {
  children: React.ReactNode;
  [key: string]: any;
};

export default function LayoutWrapper(props: LayoutProps) {
  return (
    <>
      <Layout {...props} />
    </>
  );
}
