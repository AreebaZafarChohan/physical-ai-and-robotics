import React, { useState } from 'react';
import { FaEye, FaEyeSlash } from 'react-icons/fa';

interface LoginFormProps {
  onSubmit: (formData: { email: string; password: string }) => void;
  isLoading: boolean;
  error: string | null;
}

const LoginForm: React.FC<LoginFormProps> = ({ onSubmit, isLoading, error }) => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSubmit({ email, password });
  };

  return (
    <div className="w-full max-w-md p-8 space-y-8 rounded-2xl bg-gradient-to-br from-[#0B1020] to-[#151A3D] border border-[rgba(255,255,255,0.1)] backdrop-blur-sm shadow-2xl">
      <div className="text-center">
        <h2 className="text-3xl font-bold text-white mb-2">
          Welcome Back
        </h2>
        <p className="text-gray-400">
          Sign in to your account
        </p>
      </div>
      <form onSubmit={handleSubmit} className="mt-8 space-y-6">
        <div>
          <label htmlFor="email" className="block text-sm font-medium text-gray-300 mb-2">
            Email Address
          </label>
          <input
            type="email"
            id="email"
            className="w-full px-4 py-3 rounded-lg bg-[rgba(255,255,255,0.05)] border border-[rgba(255,255,255,0.1)] text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-[#6C6CFF] focus:border-transparent transition-all"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="you@example.com"
            required
          />
        </div>
        <div>
          <label htmlFor="password" className="block text-sm font-medium text-gray-300 mb-2">
            Password
          </label>
          <div className="relative">
            <input
              type={showPassword ? "text" : "password"}
              id="password"
              className="w-full px-4 py-3 rounded-lg bg-[rgba(255,255,255,0.05)] border border-[rgba(255,255,255,0.1)] text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-[#6C6CFF] focus:border-transparent transition-all pr-12"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="••••••••"
              required
            />
            <button
              type="button"
              className="absolute inset-y-0 right-0 pr-3 flex items-center text-gray-400 hover:text-white"
              onClick={() => setShowPassword(!showPassword)}
            >
              {showPassword ? <FaEyeSlash size={20} /> : <FaEye size={20} />}
            </button>
          </div>
        </div>
        {error && (
          <div className="py-2 px-4 rounded-lg bg-red-900/30 border border-red-700/50 text-red-300">
            {error}
          </div>
        )}
        <div className="flex items-center justify-between">
          <div className="flex items-center">
            <input
              id="remember-me"
              name="remember-me"
              type="checkbox"
              className="h-4 w-4 text-[#6C6CFF] focus:ring-[#6C6CFF] border-[rgba(255,255,255,0.1)] rounded bg-[rgba(255,255,255,0.05)]"
            />
            <label htmlFor="remember-me" className="ml-2 block text-sm text-gray-300">
              Remember me
            </label>
          </div>

          <div className="text-sm">
            <a href="#" className="text-[#6C6CFF] hover:text-[#8585ff] transition-colors">
              Forgot your password?
            </a>
          </div>
        </div>
        <button
          type="submit"
          className="w-full flex justify-center py-3 px-4 border border-transparent rounded-lg shadow-sm text-sm font-medium text-white bg-gradient-to-r from-[#6C6CFF] to-[#8585ff] hover:from-[#8585ff] hover:to-[#9d9dff] focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-[#6C6CFF] transition-all duration-300"
          disabled={isLoading}
        >
          {isLoading ? (
            <span className="flex items-center">
              <svg className="animate-spin -ml-1 mr-3 h-5 w-5 text-white" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
              </svg>
              Signing In...
            </span>
          ) : (
            'Sign In'
          )}
        </button>
      </form>
      <div className="text-center text-sm text-gray-400">
        Don't have an account?{' '}
        <a href="/register" className="text-[#6C6CFF] hover:text-[#8585ff] transition-colors font-medium">
          Sign up
        </a>
      </div>
    </div>
  );
};

export default LoginForm;