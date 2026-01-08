import React, { useEffect, useMemo, useState } from 'react';
import { User, logoutUser } from '../services/auth';
import { useHistory } from '@docusaurus/router';
import { FaUser, FaEnvelope, FaSignOutAlt, FaCode, FaMicrochip, FaCog, FaTimes } from 'react-icons/fa';
import { getUserProfileDetails, updateUserProfile } from '../services/api';

interface UserProfileProps {
  user: User;
  onProfileUpdate?: (updatedUser: Partial<User>) => void;
}

type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced';

const normalizeArrayField = (v: unknown): string[] => {
  if (!v) return [];
  if (Array.isArray(v)) return v.filter(Boolean).map(String);
  return String(v)
    .split(',')
    .map((s) => s.trim())
    .filter(Boolean);
};

const UserProfile: React.FC<UserProfileProps> = ({ user, onProfileUpdate }) => {
  const history = useHistory();

  const [isEditOpen, setIsEditOpen] = useState(false);
  const [saving, setSaving] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);

  const [softwareBackground, setSoftwareBackground] = useState<string[]>([]);
  const [hardwareBackground, setHardwareBackground] = useState<string[]>([]);
  const [experienceLevel, setExperienceLevel] = useState<ExperienceLevel>('beginner');

  const softwareInput = useMemo(() => softwareBackground.join(', '), [softwareBackground]);
  const hardwareInput = useMemo(() => hardwareBackground.join(', '), [hardwareBackground]);

  const handleLogout = async () => {
    await logoutUser();
    history.push('/login'); // Redirect to login page after logout
  };

  const openEdit = async () => {
    setSaveError(null);
    setIsEditOpen(true);

    // Prefill from current user; then try to fetch more detailed profile.
    setSoftwareBackground(normalizeArrayField(user.software_background));
    setHardwareBackground(normalizeArrayField(user.hardware_background));
    setExperienceLevel((user.experience_level as ExperienceLevel) || 'beginner');

    try {
      const details = await getUserProfileDetails();
      const profile = details?.profile;
      if (profile) {
        setSoftwareBackground(normalizeArrayField(profile.software_background));
        setHardwareBackground(normalizeArrayField(profile.hardware_background));
        setExperienceLevel((profile.experience_level as ExperienceLevel) || 'beginner');
      }
    } catch (e: any) {
      // Non-blocking: user can still edit with prefills.
      console.warn('Failed to fetch profile details for edit:', e);
    }
  };

  const handleSave = async () => {
    setSaving(true);
    setSaveError(null);
    try {
      await updateUserProfile({
        software_background: softwareBackground,
        hardware_background: hardwareBackground,
        experience_level: experienceLevel,
      });

      onProfileUpdate?.({
        software_background: softwareBackground,
        hardware_background: hardwareBackground,
        experience_level: experienceLevel,
      });

      setIsEditOpen(false);
    } catch (e: any) {
      setSaveError(e?.message || 'Failed to update profile');
    } finally {
      setSaving(false);
    }
  };

  useEffect(() => {
    // Keep local edit state in sync if user prop changes.
    if (!isEditOpen) {
      setSoftwareBackground(normalizeArrayField(user.software_background));
      setHardwareBackground(normalizeArrayField(user.hardware_background));
      setExperienceLevel((user.experience_level as ExperienceLevel) || 'beginner');
    }
  }, [isEditOpen, user.experience_level, user.hardware_background, user.software_background]);

  const closeEdit = () => {
    if (!saving) setIsEditOpen(false);
  };

  const onModalKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Escape') closeEdit();
  };

  const onBackdropMouseDown = (e: React.MouseEvent<HTMLDivElement>) => {
    if (e.target === e.currentTarget) closeEdit();
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
          onClick={openEdit}
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

      {isEditOpen && (
        <div
          className="fixed inset-0 z-[2000] flex items-center justify-center px-4"
          onMouseDown={onBackdropMouseDown}
          onKeyDown={onModalKeyDown}
          role="dialog"
          aria-modal="true"
          tabIndex={-1}
        >
          <div className="absolute inset-0 bg-black/60" />

          <div className="relative w-full max-w-xl rounded-2xl border border-[rgba(108,108,255,0.25)] bg-[#0B1020]/95 backdrop-blur-xl shadow-2xl">
            <div className="flex items-center justify-between px-6 py-5 border-b border-[rgba(255,255,255,0.08)]">
              <div>
                <h3 className="text-xl font-bold text-white">Edit Profile</h3>
                <p className="text-sm text-gray-400 mt-1">Update your background and experience level.</p>
              </div>
              <button
                type="button"
                className="p-2 rounded-lg text-gray-300 hover:text-white hover:bg-white/5 transition-colors"
                onClick={closeEdit}
                aria-label="Close"
                disabled={saving}
              >
                <FaTimes className="h-4 w-4" />
              </button>
            </div>

            <div className="px-6 py-5 space-y-5">
              {saveError && (
                <div className="rounded-xl border border-red-500/30 bg-red-500/10 px-4 py-3 text-sm text-red-200">
                  {saveError}
                </div>
              )}

              <div>
                <label className="block text-sm font-medium text-gray-300 mb-2">Software background</label>
                <input
                  type="text"
                  value={softwareInput}
                  onChange={(e) => setSoftwareBackground(normalizeArrayField(e.target.value))}
                  className="w-full px-4 py-3 rounded-xl bg-[rgba(255,255,255,0.04)] border border-[rgba(255,255,255,0.1)] text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-[#6C6CFF]/50"
                  placeholder="e.g. Python, ROS2, C++"
                  disabled={saving}
                />
                <p className="text-xs text-gray-500 mt-2">Comma-separated values.</p>
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-300 mb-2">Hardware background</label>
                <input
                  type="text"
                  value={hardwareInput}
                  onChange={(e) => setHardwareBackground(normalizeArrayField(e.target.value))}
                  className="w-full px-4 py-3 rounded-xl bg-[rgba(255,255,255,0.04)] border border-[rgba(255,255,255,0.1)] text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-[#6C6CFF]/50"
                  placeholder="e.g. Arduino, Sensors, Embedded"
                  disabled={saving}
                />
                <p className="text-xs text-gray-500 mt-2">Comma-separated values.</p>
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-300 mb-2">Experience level</label>
                <select
                  value={experienceLevel}
                  onChange={(e) => setExperienceLevel(e.target.value as ExperienceLevel)}
                  className="w-full px-4 py-3 rounded-xl bg-[rgba(255,255,255,0.04)] border border-[rgba(255,255,255,0.1)] text-white focus:outline-none focus:ring-2 focus:ring-[#6C6CFF]/50"
                  disabled={saving}
                >
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                </select>
              </div>
            </div>

            <div className="px-6 py-5 border-t border-[rgba(255,255,255,0.08)] flex items-center justify-end gap-3">
              <button
                type="button"
                className="px-5 py-2.5 rounded-xl bg-white/5 text-white border border-[rgba(255,255,255,0.12)] hover:bg-white/10 transition-colors"
                onClick={closeEdit}
                disabled={saving}
              >
                Cancel
              </button>
              <button
                type="button"
                className="px-5 py-2.5 rounded-xl bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] text-white font-semibold hover:shadow-lg hover:shadow-[rgba(108,108,255,0.25)] transition-all disabled:opacity-60 disabled:cursor-not-allowed"
                onClick={handleSave}
                disabled={saving}
              >
                {saving ? 'Saving…' : 'Save changes'}
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default UserProfile;
