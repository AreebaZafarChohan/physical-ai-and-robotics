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
      <div style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        zIndex: 1000,
        backgroundColor: 'white',
        border: '1px solid #ccc',
        borderRadius: '8px',
        padding: '10px',
        boxShadow: '0 4px 8px rgba(0,0,0,0.1)'
      }}>
        <Chatbot />
      </div>
    </>
  );
}
