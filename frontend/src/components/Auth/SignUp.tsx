
import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';

const SignUp = () => {
  const [username, setUsername] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const { signup } = useAuth();

  const handleSubmit = async (event: React.FormEvent) => {
    event.preventDefault();
    setError(null);
    try {
      await signup(username, email, password);
      console.log('Signed up successfully!');
      // Optionally redirect or show success message
    } catch (err: any) {
      setError(err.message || 'Failed to sign up.');
    }
  };

  return (
    <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '50vh' }}>
      <div style={{ width: '300px', padding: '20px', border: '1px solid #ccc', borderRadius: '5px' }}>
        <h2>Sign Up</h2>
        <form onSubmit={handleSubmit}>
          <div style={{ marginBottom: '15px' }}>
            <label htmlFor="username">Username</label>
            <input
              type="text"
              id="username"
              value={username}
              onChange={(e) => setUsername(e.target.value)}
              style={{ width: '100%', padding: '8px', boxSizing: 'border-box' }}
            />
          </div>
          <div style={{ marginBottom: '15px' }}>
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              style={{ width: '100%', padding: '8px', boxSizing: 'border-box' }}
            />
          </div>
          <div style={{ marginBottom: '15px' }}>
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              style={{ width: '100%', padding: '8px', boxSizing: 'border-box' }}
            />
          </div>
          {error && <p style={{ color: 'red' }}>{error}</p>}
          <button type="submit" style={{ width: '100%', padding: '10px', backgroundColor: '#007bff', color: 'white', border: 'none', borderRadius: '5px' }}>
            Sign Up
          </button>
        </form>
      </div>
    </div>
  );
};

export default SignUp;
