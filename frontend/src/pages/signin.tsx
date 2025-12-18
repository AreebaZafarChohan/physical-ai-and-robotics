
import React from 'react';
import Layout from '@theme/Layout';
import SignIn from '@site/src/components/Auth/SignIn';

export default function SignInPage() {
  return (
    <Layout title="Sign In" description="Sign In to your account">
      <SignIn />
    </Layout>
  );
}
