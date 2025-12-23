import React from 'react';
import Layout from '@theme/Layout';

const RegistrationSuccessPage: React.FC = () => {
  return (
    <Layout
      title="Registration Success"
      description="User registration was successful."
    >
      <main className="flex items-center justify-center py-12 px-4 sm:px-6 lg:px-8">
        <div className="max-w-md w-full space-y-8 text-center">
          <h2 className="mt-6 text-3xl font-extrabold">
            Registration Successful!
          </h2>
          <p className="mt-2 text-sm">
            Thank you for registering. You can now log in to your account.
          </p>
          <a href="/login" className="font-medium text-primary hover:text-primary-light">
            Go to Login
          </a>
        </div>
      </main>
    </Layout>
  );
};

export default RegistrationSuccessPage;
