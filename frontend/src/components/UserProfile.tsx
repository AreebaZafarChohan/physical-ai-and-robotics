import React, { useState, useEffect } from 'react';
import { User, logoutUser, getCurrentUser } from '../services/auth';
import { updateUserProfile } from '../services/api';
import { useHistory } from '@docusaurus/router';
import {
  FaUser, FaEnvelope, FaSignOutAlt, FaCode, FaMicrochip,
  FaCog, FaTimes, FaSave, FaGraduationCap, FaCheck,
  FaPython, FaJs, FaReact, FaDocker, FaLinux, FaGitAlt
} from 'react-icons/fa';
import { SiRos, SiTensorflow, SiPytorch, SiCplusplus, SiArduino, SiRaspberrypi } from 'react-icons/si';

interface UserProfileProps {
  user: User;
  onProfileUpdate?: (updatedUser: User) => void;
}

// Software background options
const SOFTWARE_OPTIONS = [
  { id: 'python', label: 'Python', icon: FaPython },
  { id: 'javascript', label: 'JavaScript', icon: FaJs },
  { id: 'cpp', label: 'C/C++', icon: SiCplusplus },
  { id: 'ros2', label: 'ROS 2', icon: SiRos },
  { id: 'tensorflow', label: 'TensorFlow', icon: SiTensorflow },
  { id: 'pytorch', label: 'PyTorch', icon: SiPytorch },
  { id: 'react', label: 'React', icon: FaReact },
  { id: 'docker', label: 'Docker', icon: FaDocker },
  { id: 'linux', label: 'Linux', icon: FaLinux },
  { id: 'git', label: 'Git', icon: FaGitAlt },
];

// Hardware background options
const HARDWARE_OPTIONS = [
  { id: 'arduino', label: 'Arduino', icon: SiArduino },
  { id: 'raspberry_pi', label: 'Raspberry Pi', icon: SiRaspberrypi },
  { id: 'nvidia_jetson', label: 'NVIDIA Jetson', icon: FaMicrochip },
  { id: 'robotic_arms', label: 'Robotic Arms', icon: FaMicrochip },
  { id: 'drones', label: 'Drones/UAVs', icon: FaMicrochip },
  { id: 'sensors', label: 'Sensors & IMUs', icon: FaMicrochip },
  { id: '3d_printing', label: '3D Printing', icon: FaMicrochip },
  { id: 'pcb_design', label: 'PCB Design', icon: FaMicrochip },
];

// Experience levels
const EXPERIENCE_LEVELS = [
  { id: 'beginner', label: 'Beginner', description: 'Just starting out with robotics' },
  { id: 'intermediate', label: 'Intermediate', description: 'Some experience with robotics projects' },
  { id: 'advanced', label: 'Advanced', description: 'Extensive robotics experience' },
];

const UserProfile: React.FC<UserProfileProps> = ({ user, onProfileUpdate }) => {
  const history = useHistory();
  const [isEditing, setIsEditing] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [saveSuccess, setSaveSuccess] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);

  // Form state
  const [selectedSoftware, setSelectedSoftware] = useState<string[]>(
    Array.isArray(user.software_background) ? user.software_background : []
  );
  const [selectedHardware, setSelectedHardware] = useState<string[]>(
    Array.isArray(user.hardware_background) ? user.hardware_background : []
  );
  const [experienceLevel, setExperienceLevel] = useState<string>(
    user.experience_level || 'beginner'
  );

  const handleLogout = async () => {
    await logoutUser();
    history.push('/login');
  };

  const toggleSoftware = (id: string) => {
    setSelectedSoftware(prev =>
      prev.includes(id) ? prev.filter(s => s !== id) : [...prev, id]
    );
  };

  const toggleHardware = (id: string) => {
    setSelectedHardware(prev =>
      prev.includes(id) ? prev.filter(h => h !== id) : [...prev, id]
    );
  };

  const handleSaveProfile = async () => {
    setIsSaving(true);
    setSaveError(null);
    setSaveSuccess(false);

    try {
      // Update user profile via API service (uses dynamic URL)
      const result = await updateUserProfile({
        software_background: selectedSoftware,
        hardware_background: selectedHardware,
        experience_level: experienceLevel,
      });

      setSaveSuccess(true);
      setIsEditing(false);

      // Notify parent component
      if (onProfileUpdate) {
        onProfileUpdate(result.profile || result);
      }

      // Auto-hide success message
      setTimeout(() => setSaveSuccess(false), 3000);
    } catch (error: any) {
      setSaveError(error.message || 'Failed to save profile');
    } finally {
      setIsSaving(false);
    }
  };

  const cancelEdit = () => {
    setSelectedSoftware(Array.isArray(user.software_background) ? user.software_background : []);
    setSelectedHardware(Array.isArray(user.hardware_background) ? user.hardware_background : []);
    setExperienceLevel(user.experience_level || 'beginner');
    setIsEditing(false);
    setSaveError(null);
  };

  return (
    <div className="w-full max-w-4xl mx-auto">
      {/* Profile Header Card */}
      <div className="relative overflow-hidden bg-gradient-to-br from-[#0E1330] to-[#151A3D] rounded-3xl shadow-2xl border border-[rgba(108,108,255,0.2)] mb-6">
        {/* Background Pattern */}
        <div className="absolute inset-0 opacity-10">
          <div className="absolute top-0 right-0 w-64 h-64 bg-[#6C6CFF] rounded-full blur-3xl transform translate-x-1/2 -translate-y-1/2"></div>
          <div className="absolute bottom-0 left-0 w-48 h-48 bg-[#8a8aff] rounded-full blur-3xl transform -translate-x-1/2 translate-y-1/2"></div>
        </div>

        <div className="relative p-8">
          {/* Avatar and Basic Info */}
          <div className="flex flex-col md:flex-row items-center gap-6 mb-8">
            <div className="relative">
              <div className="w-24 h-24 rounded-full bg-gradient-to-br from-[#6C6CFF] to-[#8a8aff] p-1">
                <div className="w-full h-full rounded-full bg-[#0B1020] flex items-center justify-center">
                  <span className="text-4xl font-bold text-[#6C6CFF]">
                    {user.username?.charAt(0).toUpperCase() || 'U'}
                  </span>
                </div>
              </div>
              <div className="absolute -bottom-1 -right-1 w-8 h-8 bg-green-500 rounded-full border-4 border-[#0E1330] flex items-center justify-center">
                <FaCheck className="w-3 h-3 text-white" />
              </div>
            </div>

            <div className="text-center md:text-left flex-1">
              <h2 className="text-3xl font-bold text-white mb-2">{user.username}</h2>
              <p className="text-gray-400 flex items-center justify-center md:justify-start gap-2">
                <FaEnvelope className="w-4 h-4" />
                {user.email}
              </p>
              <div className="mt-3 flex flex-wrap gap-2 justify-center md:justify-start">
                <span className="px-3 py-1 bg-[rgba(108,108,255,0.2)] text-[#8585ff] rounded-full text-sm font-medium">
                  {experienceLevel.charAt(0).toUpperCase() + experienceLevel.slice(1)}
                </span>
                {selectedSoftware.length > 0 && (
                  <span className="px-3 py-1 bg-[rgba(34,197,94,0.2)] text-green-400 rounded-full text-sm font-medium">
                    {selectedSoftware.length} Skills
                  </span>
                )}
              </div>
            </div>

            {/* Action Buttons */}
            <div className="flex gap-3">
              {!isEditing ? (
                <button
                  onClick={() => setIsEditing(true)}
                  className="flex items-center gap-2 px-5 py-2.5 bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] text-white rounded-xl font-semibold hover:shadow-lg hover:shadow-[rgba(108,108,255,0.3)] transition-all transform hover:-translate-y-0.5"
                >
                  <FaCog className="w-4 h-4" />
                  Edit Profile
                </button>
              ) : (
                <>
                  <button
                    onClick={cancelEdit}
                    className="flex items-center gap-2 px-4 py-2.5 bg-[rgba(255,255,255,0.1)] text-white rounded-xl font-medium hover:bg-[rgba(255,255,255,0.15)] transition-all"
                  >
                    <FaTimes className="w-4 h-4" />
                    Cancel
                  </button>
                  <button
                    onClick={handleSaveProfile}
                    disabled={isSaving}
                    className="flex items-center gap-2 px-5 py-2.5 bg-gradient-to-r from-green-500 to-emerald-500 text-white rounded-xl font-semibold hover:shadow-lg hover:shadow-[rgba(34,197,94,0.3)] transition-all transform hover:-translate-y-0.5 disabled:opacity-50"
                  >
                    {isSaving ? (
                      <div className="w-4 h-4 border-2 border-white border-t-transparent rounded-full animate-spin" />
                    ) : (
                      <FaSave className="w-4 h-4" />
                    )}
                    {isSaving ? 'Saving...' : 'Save Changes'}
                  </button>
                </>
              )}
            </div>
          </div>

          {/* Success/Error Messages */}
          {saveSuccess && (
            <div className="mb-6 p-4 bg-green-500/20 border border-green-500/30 rounded-xl flex items-center gap-3 animate-fadeIn">
              <FaCheck className="w-5 h-5 text-green-400" />
              <span className="text-green-400 font-medium">Profile updated successfully!</span>
            </div>
          )}

          {saveError && (
            <div className="mb-6 p-4 bg-red-500/20 border border-red-500/30 rounded-xl flex items-center gap-3 animate-fadeIn">
              <FaTimes className="w-5 h-5 text-red-400" />
              <span className="text-red-400 font-medium">{saveError}</span>
            </div>
          )}
        </div>
      </div>

      {/* Edit Profile Form */}
      {isEditing && (
        <div className="space-y-6 animate-fadeIn">
          {/* Experience Level */}
          <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)]">
            <div className="flex items-center gap-3 mb-4">
              <div className="p-2 bg-[rgba(108,108,255,0.15)] rounded-lg">
                <FaGraduationCap className="w-5 h-5 text-[#6C6CFF]" />
              </div>
              <h3 className="text-lg font-semibold text-white">Experience Level</h3>
            </div>
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
              {EXPERIENCE_LEVELS.map(level => (
                <button
                  key={level.id}
                  onClick={() => setExperienceLevel(level.id)}
                  className={`p-4 rounded-xl border-2 transition-all text-left ${
                    experienceLevel === level.id
                      ? 'border-[#6C6CFF] bg-[rgba(108,108,255,0.15)]'
                      : 'border-[rgba(255,255,255,0.1)] hover:border-[rgba(108,108,255,0.3)]'
                  }`}
                >
                  <div className="flex items-center gap-2 mb-1">
                    {experienceLevel === level.id && (
                      <FaCheck className="w-4 h-4 text-[#6C6CFF]" />
                    )}
                    <span className="font-semibold text-white">{level.label}</span>
                  </div>
                  <p className="text-sm text-gray-400">{level.description}</p>
                </button>
              ))}
            </div>
          </div>

          {/* Software Background */}
          <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)]">
            <div className="flex items-center gap-3 mb-4">
              <div className="p-2 bg-[rgba(108,108,255,0.15)] rounded-lg">
                <FaCode className="w-5 h-5 text-[#6C6CFF]" />
              </div>
              <h3 className="text-lg font-semibold text-white">Software Background</h3>
              <span className="text-sm text-gray-400">({selectedSoftware.length} selected)</span>
            </div>
            <div className="grid grid-cols-2 md:grid-cols-5 gap-3">
              {SOFTWARE_OPTIONS.map(option => {
                const Icon = option.icon;
                const isSelected = selectedSoftware.includes(option.id);
                return (
                  <button
                    key={option.id}
                    onClick={() => toggleSoftware(option.id)}
                    className={`flex flex-col items-center gap-2 p-4 rounded-xl border-2 transition-all ${
                      isSelected
                        ? 'border-[#6C6CFF] bg-[rgba(108,108,255,0.15)]'
                        : 'border-[rgba(255,255,255,0.1)] hover:border-[rgba(108,108,255,0.3)]'
                    }`}
                  >
                    <Icon className={`w-6 h-6 ${isSelected ? 'text-[#6C6CFF]' : 'text-gray-400'}`} />
                    <span className={`text-sm font-medium ${isSelected ? 'text-white' : 'text-gray-400'}`}>
                      {option.label}
                    </span>
                  </button>
                );
              })}
            </div>
          </div>

          {/* Hardware Background */}
          <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)]">
            <div className="flex items-center gap-3 mb-4">
              <div className="p-2 bg-[rgba(108,108,255,0.15)] rounded-lg">
                <FaMicrochip className="w-5 h-5 text-[#6C6CFF]" />
              </div>
              <h3 className="text-lg font-semibold text-white">Hardware Background</h3>
              <span className="text-sm text-gray-400">({selectedHardware.length} selected)</span>
            </div>
            <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
              {HARDWARE_OPTIONS.map(option => {
                const Icon = option.icon;
                const isSelected = selectedHardware.includes(option.id);
                return (
                  <button
                    key={option.id}
                    onClick={() => toggleHardware(option.id)}
                    className={`flex flex-col items-center gap-2 p-4 rounded-xl border-2 transition-all ${
                      isSelected
                        ? 'border-[#6C6CFF] bg-[rgba(108,108,255,0.15)]'
                        : 'border-[rgba(255,255,255,0.1)] hover:border-[rgba(108,108,255,0.3)]'
                    }`}
                  >
                    <Icon className={`w-6 h-6 ${isSelected ? 'text-[#6C6CFF]' : 'text-gray-400'}`} />
                    <span className={`text-sm font-medium text-center ${isSelected ? 'text-white' : 'text-gray-400'}`}>
                      {option.label}
                    </span>
                  </button>
                );
              })}
            </div>
          </div>
        </div>
      )}

      {/* Profile Info Display (when not editing) */}
      {!isEditing && (
        <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
          {/* Software Skills Card */}
          <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)]">
            <div className="flex items-center gap-3 mb-4">
              <div className="p-2 bg-[rgba(108,108,255,0.15)] rounded-lg">
                <FaCode className="w-5 h-5 text-[#6C6CFF]" />
              </div>
              <h3 className="text-lg font-semibold text-white">Software Skills</h3>
            </div>
            {selectedSoftware.length > 0 ? (
              <div className="flex flex-wrap gap-2">
                {selectedSoftware.map(skillId => {
                  const skill = SOFTWARE_OPTIONS.find(s => s.id === skillId);
                  if (!skill) return null;
                  const Icon = skill.icon;
                  return (
                    <div
                      key={skillId}
                      className="flex items-center gap-2 px-3 py-2 bg-[rgba(108,108,255,0.1)] rounded-lg border border-[rgba(108,108,255,0.2)]"
                    >
                      <Icon className="w-4 h-4 text-[#6C6CFF]" />
                      <span className="text-sm text-white">{skill.label}</span>
                    </div>
                  );
                })}
              </div>
            ) : (
              <p className="text-gray-400 text-sm">No software skills added yet. Click "Edit Profile" to add your skills.</p>
            )}
          </div>

          {/* Hardware Skills Card */}
          <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)]">
            <div className="flex items-center gap-3 mb-4">
              <div className="p-2 bg-[rgba(108,108,255,0.15)] rounded-lg">
                <FaMicrochip className="w-5 h-5 text-[#6C6CFF]" />
              </div>
              <h3 className="text-lg font-semibold text-white">Hardware Skills</h3>
            </div>
            {selectedHardware.length > 0 ? (
              <div className="flex flex-wrap gap-2">
                {selectedHardware.map(skillId => {
                  const skill = HARDWARE_OPTIONS.find(h => h.id === skillId);
                  if (!skill) return null;
                  const Icon = skill.icon;
                  return (
                    <div
                      key={skillId}
                      className="flex items-center gap-2 px-3 py-2 bg-[rgba(108,108,255,0.1)] rounded-lg border border-[rgba(108,108,255,0.2)]"
                    >
                      <Icon className="w-4 h-4 text-[#6C6CFF]" />
                      <span className="text-sm text-white">{skill.label}</span>
                    </div>
                  );
                })}
              </div>
            ) : (
              <p className="text-gray-400 text-sm">No hardware skills added yet. Click "Edit Profile" to add your skills.</p>
            )}
          </div>
        </div>
      )}

      {/* Logout Button */}
      <div className="mt-8 text-center">
        <button
          onClick={handleLogout}
          className="inline-flex items-center gap-2 px-6 py-3 bg-[rgba(255,100,100,0.1)] text-red-400 rounded-xl font-medium hover:bg-[rgba(255,100,100,0.2)] transition-all border border-[rgba(255,100,100,0.2)]"
        >
          <FaSignOutAlt className="w-4 h-4" />
          Sign Out
        </button>
      </div>
    </div>
  );
};

export default UserProfile;
