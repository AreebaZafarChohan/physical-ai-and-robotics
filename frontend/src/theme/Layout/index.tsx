import React from 'react';
import Layout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot';

type LayoutProps = {
  children: React.ReactNode;
  [key: string]: any;
};

export default function LayoutWrapper(props: LayoutProps) {
  return (
    <>
      <Layout {...props} />
      <Chatbot />

    </>
  );
}