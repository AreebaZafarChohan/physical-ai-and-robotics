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
    <div className="auth-form-container">
      <div>
        <h2 className="auth-title">
          Create Your Account
        </h2>
        <p className="auth-subtitle">
          Join our community today
        </p>
      </div>
      <form onSubmit={handleSubmit} className="space-y-6">
        <div className="input-group">
          <label htmlFor="username" className="input-label">
            Username
          </label>
          <input
            type="text"
            id="username"
            className="input"
            value={username}
            onChange={(e) => setUsername(e.target.value)}
            placeholder="Enter your username"
            required
          />
        </div>
        <div className="input-group">
          <label htmlFor="email" className="input-label">
            Email Address
          </label>
          <input
            type="email"
            id="email"
            className="input"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="you@example.com"
            required
          />
        </div>
        <div className="input-group">
          <label htmlFor="password" className="input-label">
            Password
          </label>
          <div className="relative">
            <input
              type={showPassword ? "text" : "password"}
              id="password"
              className="input"
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
          {passwordError && (
            <p className="input-error">{passwordError}</p>
          )}
        </div>
        <div className="input-group">
          <label htmlFor="confirmPassword" className="input-label">
            Confirm Password
          </label>
          <div className="relative">
            <input
              type={showConfirmPassword ? "text" : "password"}
              id="confirmPassword"
              className="input"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              placeholder="••••••••"
              required
            />
            <button
              type="button"
              className="absolute inset-y-0 right-0 pr-3 flex items-center text-gray-400 hover:text-white"
              onClick={() => setShowConfirmPassword(!showConfirmPassword)}
            >
              {showConfirmPassword ? <FaEyeSlash size={20} /> : <FaEye size={20} />}
            </button>
          </div>
        </div>
        <div className="input-group">
          <label htmlFor="softwareBackground" className="input-label">
            Software Background
          </label>
          <input
            type="text"
            id="softwareBackground"
            className="input"
            value={softwareBackground}
            onChange={(e) => setSoftwareBackground(e.target.value)}
            placeholder="Python, JavaScript, etc. (comma-separated)"
          />
        </div>
        <div className="input-group">
          <label htmlFor="hardwareBackground" className="input-label">
            Hardware Background
          </label>
          <input
            type="text"
            id="hardwareBackground"
            className="input"
            value={hardwareBackground}
            onChange={(e) => setHardwareBackground(e.target.value)}
            placeholder="Arduino, Raspberry Pi, etc. (comma-separated)"
          />
        </div>
        {error && (
          <div className="input-error">
            {error}
          </div>
        )}
        <button
          type="submit"
          className="btn-primary"
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
      <div className="auth-footer">
        Already have an account?{' '}
        <a href="/login" className="auth-link">
          Sign in
        </a>
      </div>
    </div>
  );
};

export default SignupForm;