import React, { useState } from 'react';
import LoginForm from '../components/LoginForm';
import { loginUser } from '../services/auth';
import { useHistory } from 'react-router-dom'; // Using useHistory for older version of react-router-dom
import Layout from '@theme/Layout';

const LoginPage: React.FC = () => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const history = useHistory();

  const handleLogin = async (formData: { email: string; password: string }) => {
    setIsLoading(true);
    setError(null);
    try {
      await loginUser(formData);
      // Redirect to a dashboard or home page
      history.push('/dashboard'); // Example redirect
    } catch (err: any) {
      setError(err.message || 'Login failed. Please check your credentials.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout
      title="Login"
      description="Login to your account."
    >
      <main className="min-h-screen flex items-center justify-center bg-gradient-to-br from-[#0B1020] via-[#0E1330] to-[#151A3D] text-white p-4">
        <div className="w-full flex justify-center">
          <div className="w-full max-w-md">
            <div className="flex justify-center mb-10">
              <div className="bg-gradient-to-br from-[#6C6CFF] to-[#8a8aff] p-1 rounded-full">
                <div className="bg-[#0B1020] rounded-full p-3">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-10 w-10 text-[#6C6CFF]" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 15v2m-6 4h12a2 2 0 002-2v-6a2 2 0 00-2-2H6a2 2 0 00-2 2v6a2 2 0 002 2zm10-10V7a4 4 0 00-8 0v4h8z" />
                  </svg>
                </div>
              </div>
            </div>
            <LoginForm onSubmit={handleLogin} isLoading={isLoading} error={error} />
            <div className="mt-8 text-center text-sm text-gray-500">
              <p>By signing in, you agree to our <a href="#" className="text-[#6C6CFF] hover:underline">Terms of Service</a> and <a href="#" className="text-[#6C6CFF] hover:underline">Privacy Policy</a>.</p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default LoginPage;