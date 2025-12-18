
import React from 'react';

const SignIn = () => {
  return (
    <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '50vh' }}>
      <div style={{ width: '300px', padding: '20px', border: '1px solid #ccc', borderRadius: '5px' }}>
        <h2>Sign In</h2>
        <form>
          <div style={{ marginBottom: '15px' }}>
            <label htmlFor="email">Email</label>
            <input type="email" id="email" style={{ width: '100%', padding: '8px', boxSizing: 'border-box' }} />
          </div>
          <div style={{ marginBottom: '15px' }}>
            <label htmlFor="password">Password</label>
            <input type="password" id="password" style={{ width: '100%', padding: '8px', boxSizing: 'border-box' }} />
          </div>
          <button type="submit" style={{ width: '100%', padding: '10px', backgroundColor: '#007bff', color: 'white', border: 'none', borderRadius: '5px' }}>
            Sign In
          </button>
        </form>
      </div>
    </div>
  );
};

export default SignIn;
