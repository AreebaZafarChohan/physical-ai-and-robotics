import React, { useState } from 'react';
import SignupForm from '../components/SignupForm';
import { registerUser } from '../services/auth';
import { useHistory } from 'react-router-dom'; // Using useHistory for older version of react-router-dom
import Layout from '@theme/Layout';

const RegistrationPage: React.FC = () => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const history = useHistory();

  const handleSignup = async (formData: {
    username: string;
    email: string;
    password: string;
    software_background: string;
    hardware_background: string;
  }) => {
    setIsLoading(true);
    setError(null);
    try {
      await registerUser(formData);
      // Redirect to a success page or dashboard
      history.push('/registration-success'); // Example redirect
    } catch (err: any) {
      setError(err.message || 'Failed to register. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout
      title="Register"
      description="Create a new account."
    >
      <main className="min-h-screen auth-page bg-gradient-to-br from-[#0B1020] via-[#0E1330] to-[#151A3D] text-white p-4">
        <div className="w-full max-w-md">
          <div className="flex justify-center mb-8">
            <div className="bg-gradient-to-br from-[#6C6CFF] to-[#8a8aff] p-1 rounded-2xl">
              <div className="bg-[#0B1020] rounded-2xl p-4">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-12 w-12 text-[#6C6CFF]" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M18 9v3m0 0v3m0-3h3m-3 0h-3m-2-5a4 4 0 11-8 0 4 4 0 018 0zM3 20a6 6 0 0112 0v1H3v-1z" />
                </svg>
              </div>
            </div>
          </div>
          <SignupForm onSubmit={handleSignup} isLoading={isLoading} error={error} />
          <div className="mt-8 text-center text-sm text-gray-500">
              <p>By creating an account, you agree to our <a href="#" className="text-[#6C6CFF] hover:underline">Terms of Service</a> and <a href="#" className="text-[#6C6CFF] hover:underline">Privacy Policy</a>.</p>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default RegistrationPage;