import React, { useState } from 'react';
import SignupForm from '../components/SignupForm';
import { registerUser } from '../services/auth';
import { useHistory } from 'react-router-dom'; // Assuming react-router-dom is used for navigation

const RegistrationPage: React.FC = () => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const history = useHistory();

  const handleSignup = async (formData: {
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
    <div className="min-h-screen flex items-center justify-center bg-gray-50 py-12 px-4 sm:px-6 lg:px-8">
      <div className="max-w-md w-full space-y-8">
        <div>
          <h2 className="mt-6 text-center text-3xl font-extrabold text-gray-900">
            Create your account
          </h2>
        </div>
        <SignupForm onSubmit={handleSignup} isLoading={isLoading} error={error} />
      </div>
    </div>
  );
};

export default RegistrationPage;