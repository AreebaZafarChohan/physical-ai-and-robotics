import React, { useState } from 'react';
import LoginForm from '../components/LoginForm';
import { loginUser } from '../services/auth';
import { useHistory } from 'react-router-dom'; // Assuming react-router-dom is used for navigation

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
    <div className="min-h-screen flex items-center justify-center bg-gray-50 py-12 px-4 sm:px-6 lg:px-8">
      <div className="max-w-md w-full space-y-8">
        <div>
          <h2 className="mt-6 text-center text-3xl font-extrabold text-gray-900">
            Sign in to your account
          </h2>
        </div>
        <LoginForm onSubmit={handleLogin} isLoading={isLoading} error={error} />
      </div>
    </div>
  );
};

export default LoginPage;