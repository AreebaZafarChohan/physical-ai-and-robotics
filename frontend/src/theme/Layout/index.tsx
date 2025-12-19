import React from 'react';
import Layout from '@theme-original/Layout';
import type {Props} from '@docusaurus/theme-classic/lib/theme/Layout';
import HighlightMenu from '@site/src/components/HighlightMenu';
import Chatbot from '@site/src/components/Chatbot';

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <>
      <Layout {...props}>
        {props.children}
      </Layout>
      <HighlightMenu />
      <Chatbot/>
    </>
  );
}
