import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import UserProfile from '../components/UserProfile';
import { getCurrentUser } from '../services/auth';
import { useHistory } from '@docusaurus/router';
import { FaRunning, FaChartLine, FaUsers, FaBrain, FaMicrochip, FaCog } from 'react-icons/fa';

const DashboardPage: React.FC = () => {
  const [user, setUser] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const history = useHistory();

  useEffect(() => {
    const fetchUser = async () => {
      try {
        const currentUser = await getCurrentUser();
        if (currentUser) {
          setUser(currentUser);
        } else {
          // If no current user, redirect to login page
          history.push('/LoginPage');
          return;
        }
      } catch (err: any) {
        setError(err.message || 'Failed to fetch user data.');
      } finally {
        setLoading(false);
      }
    };

    fetchUser();
  }, [history]); // Add history to dependency array

  if (loading) {
    return (
      <Layout title="Dashboard" description="User Dashboard">
        <main className="min-h-screen flex items-center justify-center bg-gradient-to-br from-[#0B1020] via-[#0E1330] to-[#151A3D] text-white">
          <div className="text-center">
            <div className="flex justify-center mb-4">
              <div className="w-16 h-16 border-4 border-[#6C6CFF] border-t-transparent rounded-full animate-spin"></div>
            </div>
            <h2 className="text-2xl font-semibold">Loading your dashboard...</h2>
          </div>
        </main>
      </Layout>
    );
  }

  if (error) {
    return (
      <Layout title="Dashboard" description="User Dashboard">
        <main className="min-h-screen flex items-center justify-center bg-gradient-to-br from-[#0B1020] via-[#0E1330] to-[#151A3D] text-white">
          <div className="text-center max-w-md p-6 bg-[#0E1330] rounded-2xl shadow-xl border border-[rgba(255,255,255,0.1)] backdrop-blur-sm">
            <div className="mx-auto bg-red-900/30 w-16 h-16 rounded-full flex items-center justify-center mb-4">
              <FaRunning className="h-8 w-8 text-red-400" />
            </div>
            <h2 className="text-xl font-bold text-white mb-2">Error Loading Dashboard</h2>
            <p className="text-red-400 mb-4">{error}</p>
            <button
              onClick={() => history.push('/login')}
              className="px-4 py-2 bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] text-white rounded-lg hover:from-[#8585ff] hover:to-[#9d9dff] transition-all"
            >
              Go to Login
            </button>
          </div>
        </main>
      </Layout>
    );
  }

  // Fallback if user is somehow null after loading and no error (should be redirected by now)
  if (!user) {
    return (
      <Layout title="Dashboard" description="User Dashboard">
        <main className="min-h-screen flex items-center justify-center bg-gradient-to-br from-[#0B1020] via-[#0E1330] to-[#151A3D] text-white">
          <div className="text-center max-w-md p-6 bg-[#0E1330] rounded-2xl shadow-xl border border-[rgba(255,255,255,0.1)] backdrop-blur-sm">
            <div className="mx-auto bg-red-900/30 w-16 h-16 rounded-full flex items-center justify-center mb-4">
              <FaRunning className="h-8 w-8 text-red-400" />
            </div>
            <h2 className="text-xl font-bold text-white mb-2">Access Denied</h2>
            <p className="text-gray-400 mb-4">You need to be logged in to view this page.</p>
            <button
              onClick={() => history.push('/login')}
              className="px-4 py-2 bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] text-white rounded-lg hover:from-[#8585ff] hover:to-[#9d9dff] transition-all"
            >
              Go to Login
            </button>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title="Dashboard" description="User Dashboard">
      <main className="min-h-screen bg-gradient-to-br from-[#0B1020] via-[#0E1330] to-[#151A3D] text-white">
        {/* Header */}
        <div className="bg-[#0B1020]/80 backdrop-blur-lg border-b border-[rgba(255,255,255,0.1)]">
          <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
            <div className="flex items-center justify-between h-16">
              <div className="flex items-center">
                <FaBrain className="h-8 w-8 text-[#6C6CFF] mr-3" />
                <h1 className="text-xl font-bold">AI Robotics Dashboard</h1>
              </div>
              <div className="flex items-center space-x-4">
                <button className="p-2 rounded-full hover:bg-[rgba(255,255,255,0.1)] transition-colors">
                  <FaCog className="h-5 w-5 text-gray-300" />
                </button>
                <div className="h-8 w-8 rounded-full bg-gradient-to-br from-[#6C6CFF] to-[#8a8aff] flex items-center justify-center text-xs font-bold">
                  {user.username.charAt(0).toUpperCase()}
                </div>
              </div>
            </div>
          </div>
        </div>

        <div className="max-w-7xl mx-auto py-8 px-4 sm:px-6 lg:px-8">
          <div className="mb-10">
            <h2 className="text-3xl font-bold text-white mb-2">
              Welcome back, <span className="text-[#6C6CFF]">{user.username}</span>!
            </h2>
            <p className="text-gray-400">Here's what's happening with your account today.</p>
          </div>

          {/* Stats Cards */}
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-10">
            <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)] backdrop-blur-sm">
              <div className="flex items-center">
                <div className="p-3 rounded-lg bg-[rgba(108,108,255,0.1)]">
                  <FaRunning className="h-6 w-6 text-[#6C6CFF]" />
                </div>
                <div className="ml-4">
                  <p className="text-sm font-medium text-gray-400">Active Sessions</p>
                  <p className="text-2xl font-bold text-white">4</p>
                </div>
              </div>
            </div>

            <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)] backdrop-blur-sm">
              <div className="flex items-center">
                <div className="p-3 rounded-lg bg-[rgba(108,108,255,0.1)]">
                  <FaChartLine className="h-6 w-6 text-[#6C6CFF]" />
                </div>
                <div className="ml-4">
                  <p className="text-sm font-medium text-gray-400">Projects</p>
                  <p className="text-2xl font-bold text-white">12</p>
                </div>
              </div>
            </div>

            <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)] backdrop-blur-sm">
              <div className="flex items-center">
                <div className="p-3 rounded-lg bg-[rgba(108,108,255,0.1)]">
                  <FaMicrochip className="h-6 w-6 text-[#6C6CFF]" />
                </div>
                <div className="ml-4">
                  <p className="text-sm font-medium text-gray-400">Robots</p>
                  <p className="text-2xl font-bold text-white">3</p>
                </div>
              </div>
            </div>

            <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)] backdrop-blur-sm">
              <div className="flex items-center">
                <div className="p-3 rounded-lg bg-[rgba(108,108,255,0.1)]">
                  <FaUsers className="h-6 w-6 text-[#6C6CFF]" />
                </div>
                <div className="ml-4">
                  <p className="text-sm font-medium text-gray-400">Team Members</p>
                  <p className="text-2xl font-bold text-white">8</p>
                </div>
              </div>
            </div>
          </div>

          {/* User Profile Card */}
          <div className="mb-10">
            <UserProfile user={user} />
          </div>

          {/* Recent Activity */}
          <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)] backdrop-blur-sm">
            <h3 className="text-xl font-bold text-white mb-4">Recent Activity</h3>
            <div className="space-y-4">
              <div className="flex items-start">
                <div className="flex-shrink-0 mt-1">
                  <div className="w-2 h-2 rounded-full bg-[#6C6CFF]"></div>
                </div>
                <div className="ml-3">
                  <p className="text-white">Deployed new firmware to <span className="text-[#6C6CFF]">RoboArm-v3</span></p>
                  <p className="text-sm text-gray-400">2 hours ago</p>
                </div>
              </div>
              <div className="flex items-start">
                <div className="flex-shrink-0 mt-1">
                  <div className="w-2 h-2 rounded-full bg-[#6C6CFF]"></div>
                </div>
                <div className="ml-3">
                  <p className="text-white">Completed <span className="text-[#6C6CFF]">Path Planning Simulation</span> for Project Titan</p>
                  <p className="text-sm text-gray-400">5 hours ago</p>
                </div>
              </div>
              <div className="flex items-start">
                <div className="flex-shrink-0 mt-1">
                  <div className="w-2 h-2 rounded-full bg-[#6C6CFF]"></div>
                </div>
                <div className="ml-3">
                  <p className="text-white">Started collaboration with <span className="text-[#6C6CFF]">Team Alpha</span></p>
                  <p className="text-sm text-gray-400">Yesterday</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default DashboardPage;
