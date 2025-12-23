import React from 'react';
import { User, logoutUser } from '../services/auth';
import { useHistory } from '@docusaurus/router';

interface UserProfileProps {
  user: User;
}

const UserProfile: React.FC<UserProfileProps> = ({ user }) => {
  const history = useHistory();

  const handleLogout = async () => {
    await logoutUser();
    history.push('/login'); // Redirect to login page after logout
  };

  return (
    <div className="card mx-auto p-6 rounded-lg shadow-lg max-w-md">
      <h3 className="text-2xl font-bold text-center mb-4">User Profile</h3>
      <div className="space-y-3">
        <p><strong>Username:</strong> {user.username}</p>
        <p><strong>Email:</strong> {user.email}</p>
        {/* Add other user details here as needed */}
      </div>
      <button
        onClick={handleLogout}
        className="btn-primary w-full mt-6"
      >
        Logout
      </button>
    </div>
  );
};

export default UserProfile;
