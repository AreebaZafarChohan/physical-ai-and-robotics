
import React from 'react';
import Layout from '@theme/Layout';
import SignUp from '@site/src/components/Auth/SignUp';

export default function SignUpPage() {
  return (
    <Layout title="Sign Up" description="Create a new account">
      <SignUp />
    </Layout>
  );
}
