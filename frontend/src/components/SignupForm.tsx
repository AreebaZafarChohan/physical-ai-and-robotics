import React, { useState } from 'react';
import { FaEye, FaEyeSlash } from 'react-icons/fa';

interface SignupFormProps {
  onSubmit: (formData: {
    username: string;
    email: string;
    password: string;
    software_background: string;
    hardware_background: string;
  }) => void;
  isLoading: boolean;
  error: string | null;
}

const SignupForm: React.FC<SignupFormProps> = ({ onSubmit, isLoading, error }) => {
  const [username, setUsername] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
  const [passwordError, setPasswordError] = useState<string | null>(null);

  const validatePassword = (pw: string) => {
    if (pw.length < 8) return 'Password must be at least 8 characters long.';
    if (!/[A-Z]/.test(pw)) return 'Password must contain at least one uppercase letter.';
    if (!/[a-z]/.test(pw)) return 'Password must contain at least one lowercase letter.';
    if (!/[0-9]/.test(pw)) return 'Password must contain at least one number.';
    return null;
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    setPasswordError(null);

    const pwValidationError = validatePassword(password);
    if (pwValidationError) {
      setPasswordError(pwValidationError);
      return;
    }

    if (password !== confirmPassword) {
      setPasswordError('Passwords do not match.');
      return;
    }

    onSubmit({
      username,
      email,
      password,
      software_background: softwareBackground,
      hardware_background: hardwareBackground,
    });
  };

  return (
    <div className="w-full max-w-md p-8 space-y-8 bg-[#0E1330] rounded-2xl shadow-xl border border-[rgba(255,255,255,0.1)] backdrop-blur-sm">
      <div>
        <h2 className="mt-6 text-center text-3xl font-extrabold text-white">
          Create Your Account
        </h2>
        <p className="mt-2 text-center text-sm text-gray-400">
          Join our community today
        </p>
      </div>
      <form onSubmit={handleSubmit} className="space-y-6">
        <div>
          <label htmlFor="username" className="block text-sm font-medium text-gray-300 mb-2">
            Username
          </label>
          <input
            type="text"
            id="username"
            className="w-full px-4 py-3 bg-[rgba(255,255,255,0.05)] border border-[rgba(255,255,255,0.1)] rounded-lg text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-[#6C6CFF] focus:border-[#6C6CFF] transition-all"
            value={username}
            onChange={(e) => setUsername(e.target.value)}
            placeholder="Enter your username"
            required
          />
        </div>
        <div>
          <label htmlFor="email" className="block text-sm font-medium text-gray-300 mb-2">
            Email Address
          </label>
          <input
            type="email"
            id="email"
            className="w-full px-4 py-3 bg-[rgba(255,255,255,0.05)] border border-[rgba(255,255,255,0.1)] rounded-lg text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-[#6C6CFF] focus:border-[#6C6CFF] transition-all"
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
              className="w-full px-4 py-3 bg-[rgba(255,255,255,0.05)] border border-[rgba(255,255,255,0.1)] rounded-lg text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-[#6C6CFF] focus:border-[#6C6CFF] transition-all pr-12"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="••••••••"
              required
            />
            <button
              type="button"
              className="absolute inset-y-0 right-0 pr-3 flex items-center text-gray-500 hover:text-white"
              onClick={() => setShowPassword(!showPassword)}
            >
              {showPassword ? <FaEyeSlash size={20} /> : <FaEye size={20} />}
            </button>
          </div>
          {passwordError && (
            <p className="mt-2 text-xs text-red-400">{passwordError}</p>
          )}
        </div>
        <div>
          <label htmlFor="confirmPassword" className="block text-sm font-medium text-gray-300 mb-2">
            Confirm Password
          </label>
          <div className="relative">
            <input
              type={showConfirmPassword ? "text" : "password"}
              id="confirmPassword"
              className="w-full px-4 py-3 bg-[rgba(255,255,255,0.05)] border border-[rgba(255,255,255,0.1)] rounded-lg text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-[#6C6CFF] focus:border-[#6C6CFF] transition-all pr-12"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              placeholder="••••••••"
              required
            />
            <button
              type="button"
              className="absolute inset-y-0 right-0 pr-3 flex items-center text-gray-500 hover:text-white"
              onClick={() => setShowConfirmPassword(!showConfirmPassword)}
            >
              {showConfirmPassword ? <FaEyeSlash size={20} /> : <FaEye size={20} />}
            </button>
          </div>
        </div>
        <div>
          <label htmlFor="softwareBackground" className="block text-sm font-medium text-gray-300 mb-2">
            Software Background
          </label>
          <input
            type="text"
            id="softwareBackground"
            className="w-full px-4 py-3 bg-[rgba(255,255,255,0.05)] border border-[rgba(255,255,255,0.1)] rounded-lg text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-[#6C6CFF] focus:border-[#6C6CFF] transition-all"
            value={softwareBackground}
            onChange={(e) => setSoftwareBackground(e.target.value)}
            placeholder="Python, JavaScript, etc. (comma-separated)"
          />
        </div>
        <div>
          <label htmlFor="hardwareBackground" className="block text-sm font-medium text-gray-300 mb-2">
            Hardware Background
          </label>
          <input
            type="text"
            id="hardwareBackground"
            className="w-full px-4 py-3 bg-[rgba(255,255,255,0.05)] border border-[rgba(255,255,255,0.1)] rounded-lg text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-[#6C6CFF] focus:border-[#6C6CFF] transition-all"
            value={hardwareBackground}
            onChange={(e) => setHardwareBackground(e.target.value)}
            placeholder="Arduino, Raspberry Pi, etc. (comma-separated)"
          />
        </div>
        {error && (
          <div className="rounded-md bg-red-900/30 p-4 border border-red-700/50">
            <div className="text-sm text-red-300">{error}</div>
          </div>
        )}
        <button
          type="submit"
          className="w-full flex justify-center py-3 px-4 border border-transparent rounded-lg shadow-sm text-sm font-medium text-white bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] hover:from-[#8585ff] hover:to-[#9d9dff] focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-[#6C6CFF] transition-all transform hover:scale-[1.02] disabled:opacity-50 disabled:cursor-not-allowed"
          disabled={isLoading}
        >
          {isLoading ? (
            <span className="flex items-center">
              <svg className="animate-spin -ml-1 mr-3 h-5 w-5 text-white" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
              </svg>
              Creating Account...
            </span>
          ) : (
            'Create Account'
          )}
        </button>
      </form>
      <div className="text-center text-sm text-gray-500">
        Already have an account?{' '}
        <a href="/login" className="font-medium text-[#6C6CFF] hover:text-[#8585ff] transition-colors">
          Sign in
        </a>
      </div>
    </div>
  );
};

export default SignupForm;