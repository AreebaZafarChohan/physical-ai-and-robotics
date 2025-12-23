import React from 'react';
import { User, logoutUser } from '../services/auth';
import { useHistory } from '@docusaurus/router';
import { FaUser, FaEnvelope, FaSignOutAlt, FaCode, FaMicrochip, FaCog } from 'react-icons/fa';

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
    <div className="w-full max-w-2xl mx-auto p-8 bg-[#0E1330] rounded-2xl shadow-xl border border-[rgba(255,255,255,0.1)] backdrop-blur-sm">
      <div className="flex items-center justify-center mb-8">
        <div className="bg-gradient-to-br from-[#6C6CFF] to-[#8a8aff] p-1 rounded-full">
          <div className="bg-[#0B1020] rounded-full p-2">
            <FaUser className="h-12 w-12 text-[#6C6CFF]" />
          </div>
        </div>
      </div>

      <h2 className="text-2xl font-bold text-center text-white mb-8">Your Profile</h2>

      <div className="space-y-6">
        <div className="flex items-center p-4 bg-[rgba(255,255,255,0.05)] rounded-xl border border-[rgba(255,255,255,0.1)]">
          <div className="mr-4 p-2 bg-[rgba(108,108,255,0.1)] rounded-lg">
            <FaUser className="h-6 w-6 text-[#6C6CFF]" />
          </div>
          <div>
            <h3 className="text-sm font-medium text-gray-400">Username</h3>
            <p className="text-lg font-semibold text-white">{user.username}</p>
          </div>
        </div>

        <div className="flex items-center p-4 bg-[rgba(255,255,255,0.05)] rounded-xl border border-[rgba(255,255,255,0.1)]">
          <div className="mr-4 p-2 bg-[rgba(108,108,255,0.1)] rounded-lg">
            <FaEnvelope className="h-6 w-6 text-[#6C6CFF]" />
          </div>
          <div>
            <h3 className="text-sm font-medium text-gray-400">Email</h3>
            <p className="text-lg font-semibold text-white">{user.email}</p>
          </div>
        </div>

        {user.software_background && (
          <div className="p-4 bg-[rgba(255,255,255,0.05)] rounded-xl border border-[rgba(255,255,255,0.1)]">
            <div className="flex items-center mb-2">
              <div className="mr-4 p-2 bg-[rgba(108,108,255,0.1)] rounded-lg">
                <FaCode className="h-6 w-6 text-[#6C6CFF]" />
              </div>
              <h3 className="text-sm font-medium text-gray-400">Software Background</h3>
            </div>
            <p className="text-white">{user.software_background}</p>
          </div>
        )}

        {user.hardware_background && (
          <div className="p-4 bg-[rgba(255,255,255,0.05)] rounded-xl border border-[rgba(255,255,255,0.1)]">
            <div className="flex items-center mb-2">
              <div className="mr-4 p-2 bg-[rgba(108,108,255,0.1)] rounded-lg">
                <FaMicrochip className="h-6 w-6 text-[#6C6CFF]" />
              </div>
              <h3 className="text-sm font-medium text-gray-400">Hardware Background</h3>
            </div>
            <p className="text-white">{user.hardware_background}</p>
          </div>
        )}
      </div>

      <div className="mt-8 flex justify-center space-x-4">
        <button
          onClick={() => {}}
          className="flex items-center px-4 py-2 bg-[rgba(255,255,255,0.05)] text-white rounded-lg border border-[rgba(255,255,255,0.1)] hover:bg-[rgba(255,255,255,0.1)] transition-colors"
        >
          <FaCog className="h-5 w-5 mr-2" />
          Edit Profile
        </button>
        <button
          onClick={handleLogout}
          className="flex items-center px-4 py-2 bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] text-white rounded-lg hover:from-[#8585ff] hover:to-[#9d9dff] transition-all"
        >
          <FaSignOutAlt className="h-5 w-5 mr-2" />
          Logout
        </button>
      </div>
    </div>
  );
};

export default UserProfile;
