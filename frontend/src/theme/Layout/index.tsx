import React from 'react';
import Layout from '@theme-original/Layout';
import Chatbot from '../../components/Chatbot'; // Import the Chatbot component

type LayoutProps = {
  children: React.ReactNode;
  [key: string]: any;
};

export default function LayoutWrapper(props: LayoutProps) {
  return (
    <>
      <Layout {...props} />
      {/* Integrate the Chatbot component here */}
      <Chatbot />
    </>
  );
}
