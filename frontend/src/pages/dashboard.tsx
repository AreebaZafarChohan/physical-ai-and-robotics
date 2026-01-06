import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import UserProfile from '../components/UserProfile';
import { getCurrentUser } from '../services/auth';
import { useHistory } from '@docusaurus/router';
import {
  FaRunning, FaChartLine, FaUsers, FaBrain, FaMicrochip, FaCog,
  FaBook, FaRobot, FaGraduationCap, FaTrophy, FaArrowRight,
  FaCheckCircle, FaClock, FaFire
} from 'react-icons/fa';

const DashboardPage: React.FC = () => {
  const [user, setUser] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [activeTab, setActiveTab] = useState<'overview' | 'profile' | 'progress'>('overview');
  const history = useHistory();

  useEffect(() => {
    const fetchUser = async () => {
      try {
        const currentUser = await getCurrentUser();
        if (currentUser) {
          setUser(currentUser);
        } else {
          history.push('/login');
          return;
        }
      } catch (err: any) {
        setError(err.message || 'Failed to fetch user data.');
      } finally {
        setLoading(false);
      }
    };

    fetchUser();
  }, [history]);

  const handleProfileUpdate = (updatedUser: any) => {
    setUser({ ...user, ...updatedUser });
  };

  if (loading) {
    return (
      <Layout title="Dashboard" description="User Dashboard">
        <main className="min-h-screen flex items-center justify-center bg-gradient-to-br from-[#0B1020] via-[#0E1330] to-[#151A3D] text-white">
          <div className="text-center">
            <div className="relative">
              <div className="w-20 h-20 border-4 border-[#6C6CFF]/30 rounded-full"></div>
              <div className="absolute top-0 left-0 w-20 h-20 border-4 border-[#6C6CFF] border-t-transparent rounded-full animate-spin"></div>
            </div>
            <h2 className="mt-6 text-2xl font-semibold bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] bg-clip-text text-transparent">
              Loading your dashboard...
            </h2>
            <p className="mt-2 text-gray-400">Please wait a moment</p>
          </div>
        </main>
      </Layout>
    );
  }

  if (error || !user) {
    return (
      <Layout title="Dashboard" description="User Dashboard">
        <main className="min-h-screen flex items-center justify-center bg-gradient-to-br from-[#0B1020] via-[#0E1330] to-[#151A3D] text-white">
          <div className="text-center max-w-md p-8 bg-[#0E1330] rounded-3xl shadow-2xl border border-[rgba(255,255,255,0.1)]">
            <div className="mx-auto w-20 h-20 bg-red-900/30 rounded-full flex items-center justify-center mb-6">
              <FaRunning className="h-10 w-10 text-red-400" />
            </div>
            <h2 className="text-2xl font-bold text-white mb-3">
              {error ? 'Error Loading Dashboard' : 'Access Denied'}
            </h2>
            <p className="text-gray-400 mb-6">
              {error || 'You need to be logged in to view this page.'}
            </p>
            <button
              onClick={() => history.push('/login')}
              className="px-6 py-3 bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] text-white rounded-xl font-semibold hover:shadow-lg hover:shadow-[rgba(108,108,255,0.3)] transition-all"
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
        {/* Hero Header */}
        <div className="relative overflow-hidden">
          {/* Background decorations */}
          <div className="absolute inset-0 overflow-hidden">
            <div className="absolute top-0 right-0 w-96 h-96 bg-[#6C6CFF]/10 rounded-full blur-3xl transform translate-x-1/2 -translate-y-1/2"></div>
            <div className="absolute bottom-0 left-0 w-64 h-64 bg-[#8a8aff]/10 rounded-full blur-3xl transform -translate-x-1/2 translate-y-1/2"></div>
          </div>

          <div className="relative bg-[#0B1020]/60 backdrop-blur-xl border-b border-[rgba(255,255,255,0.05)]">
            <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
              <div className="flex flex-col md:flex-row items-start md:items-center justify-between gap-6">
                {/* Welcome Section */}
                <div className="flex items-center gap-6">
                  <div className="relative">
                    <div className="w-20 h-20 rounded-2xl bg-gradient-to-br from-[#6C6CFF] to-[#8a8aff] p-0.5">
                      <div className="w-full h-full rounded-2xl bg-[#0B1020] flex items-center justify-center">
                        <span className="text-3xl font-bold text-[#6C6CFF]">
                          {user.username?.charAt(0).toUpperCase() || 'U'}
                        </span>
                      </div>
                    </div>
                    <div className="absolute -bottom-1 -right-1 w-6 h-6 bg-green-500 rounded-full border-2 border-[#0B1020] flex items-center justify-center">
                      <FaCheckCircle className="w-3 h-3 text-white" />
                    </div>
                  </div>
                  <div>
                    <p className="text-gray-400 text-sm mb-1">Welcome back,</p>
                    <h1 className="text-3xl font-bold">
                      <span className="bg-gradient-to-r from-white to-gray-300 bg-clip-text text-transparent">
                        {user.username}
                      </span>
                    </h1>
                    <p className="text-gray-400 text-sm mt-1 flex items-center gap-2">
                      <FaClock className="w-3 h-3" />
                      Last active: Today
                    </p>
                  </div>
                </div>

                {/* Quick Actions */}
                <div className="flex gap-3">
                  <button
                    onClick={() => history.push('/docs/intro')}
                    className="flex items-center gap-2 px-5 py-2.5 bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] text-white rounded-xl font-semibold hover:shadow-lg hover:shadow-[rgba(108,108,255,0.3)] transition-all"
                  >
                    <FaBook className="w-4 h-4" />
                    Continue Learning
                  </button>
                  <button
                    onClick={() => setActiveTab('profile')}
                    className="flex items-center gap-2 px-5 py-2.5 bg-[rgba(255,255,255,0.05)] text-white rounded-xl font-medium hover:bg-[rgba(255,255,255,0.1)] transition-all border border-[rgba(255,255,255,0.1)]"
                  >
                    <FaCog className="w-4 h-4" />
                    Settings
                  </button>
                </div>
              </div>

              {/* Navigation Tabs */}
              <div className="mt-8">
                <div className="inline-flex gap-1 p-1 rounded-2xl bg-[rgba(255,255,255,0.04)] border border-[rgba(255,255,255,0.08)]">
                  {[
                    { id: 'overview', label: 'Overview', icon: FaChartLine },
                    { id: 'profile', label: 'Profile', icon: FaUsers },
                    { id: 'progress', label: 'Progress', icon: FaTrophy },
                  ].map((tab) => {
                    const Icon = tab.icon;
                    const active = activeTab === tab.id;
                    return (
                      <button
                        key={tab.id}
                        onClick={() => setActiveTab(tab.id as any)}
                        className={`flex items-center gap-2 px-4 py-2.5 rounded-xl font-medium transition-all ${
                          active
                            ? 'bg-gradient-to-r from-[#6C6CFF]/25 to-[#8a8aff]/20 text-white shadow-sm'
                            : 'text-gray-400 hover:text-white hover:bg-white/5'
                        }`}
                      >
                        <Icon className={`w-4 h-4 ${active ? 'text-[#8a8aff]' : ''}`} />
                        {tab.label}
                      </button>
                    );
                  })}
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Main Content */}
        <div className="max-w-7xl mx-auto py-8 px-4 sm:px-6 lg:px-8">
          {activeTab === 'overview' && (
            <div className="space-y-8 animate-fadeIn">
              {/* Stats Cards */}
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
                {[
                  { label: 'Chapters Completed', value: '8', icon: FaCheckCircle, color: 'from-green-500 to-emerald-500', bgColor: 'rgba(34,197,94,0.1)' },
                  { label: 'Learning Streak', value: '5 days', icon: FaFire, color: 'from-orange-500 to-red-500', bgColor: 'rgba(249,115,22,0.1)' },
                  { label: 'Quizzes Passed', value: '12', icon: FaTrophy, color: 'from-yellow-500 to-amber-500', bgColor: 'rgba(234,179,8,0.1)' },
                  { label: 'Hours Learned', value: '24', icon: FaClock, color: 'from-blue-500 to-cyan-500', bgColor: 'rgba(59,130,246,0.1)' },
                ].map((stat, index) => {
                  const Icon = stat.icon;
                  return (
                    <div
                      key={index}
                      className="group relative overflow-hidden bg-[#0E1330]/80 backdrop-blur-sm rounded-2xl p-6 border border-[rgba(255,255,255,0.1)] hover:border-[rgba(108,108,255,0.35)] transition-all hover:shadow-xl hover:shadow-[rgba(108,108,255,0.12)]"
                    >
                      <div className="flex items-start justify-between">
                        <div>
                          <p className="text-sm font-medium text-gray-400 mb-1">{stat.label}</p>
                          <p className="text-3xl font-bold text-white">{stat.value}</p>
                        </div>
                        <div className={`p-3 rounded-xl`} style={{ background: stat.bgColor }}>
                          <Icon className={`h-6 w-6 bg-gradient-to-r ${stat.color} bg-clip-text text-transparent`} style={{ color: stat.color.includes('green') ? '#22c55e' : stat.color.includes('orange') ? '#f97316' : stat.color.includes('yellow') ? '#eab308' : '#3b82f6' }} />
                        </div>
                      </div>
                      {/* Hover effect */}
                      <div className="absolute inset-0 bg-gradient-to-r from-[#6C6CFF]/0 to-[#6C6CFF]/0 group-hover:from-[#6C6CFF]/5 group-hover:to-transparent transition-all"></div>
                    </div>
                  );
                })}
              </div>

              {/* Quick Access Cards */}
              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
                {/* Continue Learning */}
                <div className="bg-gradient-to-br from-[#0E1330] to-[#151A3D] rounded-2xl p-6 border border-[rgba(108,108,255,0.2)]">
                  <div className="flex items-center gap-3 mb-6">
                    <div className="p-3 bg-[rgba(108,108,255,0.15)] rounded-xl">
                      <FaBook className="h-6 w-6 text-[#6C6CFF]" />
                    </div>
                    <h3 className="text-xl font-bold text-white">Continue Learning</h3>
                  </div>

                  <div className="space-y-4">
                    {[
                      { title: 'Introduction to ROS 2', progress: 75, module: 'Module 1' },
                      { title: 'Digital Twin Basics', progress: 30, module: 'Module 2' },
                    ].map((course, i) => (
                      <div key={i} className="p-4 bg-[rgba(255,255,255,0.03)] rounded-xl hover:bg-[rgba(255,255,255,0.05)] transition-all cursor-pointer group">
                        <div className="flex items-center justify-between mb-2">
                          <div>
                            <p className="text-sm text-gray-400">{course.module}</p>
                            <p className="text-white font-medium">{course.title}</p>
                          </div>
                          <FaArrowRight className="w-4 h-4 text-gray-500 group-hover:text-[#6C6CFF] group-hover:translate-x-1 transition-all" />
                        </div>
                        <div className="w-full h-2 bg-[rgba(255,255,255,0.1)] rounded-full overflow-hidden">
                          <div
                            className="h-full bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] rounded-full transition-all"
                            style={{ width: `${course.progress}%` }}
                          ></div>
                        </div>
                        <p className="text-xs text-gray-500 mt-1">{course.progress}% complete</p>
                      </div>
                    ))}
                  </div>

                  <button
                    onClick={() => history.push('/docs/intro')}
                    className="mt-6 w-full py-3 bg-[rgba(108,108,255,0.1)] text-[#6C6CFF] rounded-xl font-medium hover:bg-[rgba(108,108,255,0.2)] transition-all"
                  >
                    View All Modules
                  </button>
                </div>

                {/* Recent Activity */}
                <div className="bg-[#0E1330] rounded-2xl p-6 border border-[rgba(255,255,255,0.1)]">
                  <div className="flex items-center gap-3 mb-6">
                    <div className="p-3 bg-[rgba(108,108,255,0.15)] rounded-xl">
                      <FaClock className="h-6 w-6 text-[#6C6CFF]" />
                    </div>
                    <h3 className="text-xl font-bold text-white">Recent Activity</h3>
                  </div>

                  <div className="space-y-4">
                    {[
                      { action: 'Completed chapter', target: 'Nodes, Topics & Services', time: '2 hours ago', icon: FaCheckCircle, color: 'text-green-400' },
                      { action: 'Passed quiz', target: 'ROS 2 Basics Quiz', time: '5 hours ago', icon: FaTrophy, color: 'text-yellow-400' },
                      { action: 'Started learning', target: 'RCLPY Integration', time: 'Yesterday', icon: FaBook, color: 'text-blue-400' },
                      { action: 'Updated profile', target: 'Added new skills', time: '2 days ago', icon: FaUsers, color: 'text-purple-400' },
                    ].map((activity, i) => {
                      const Icon = activity.icon;
                      return (
                        <div key={i} className="flex items-start gap-4">
                          <div className={`mt-1 ${activity.color}`}>
                            <Icon className="w-4 h-4" />
                          </div>
                          <div className="flex-1">
                            <p className="text-white">
                              {activity.action}{' '}
                              <span className="text-[#6C6CFF]">{activity.target}</span>
                            </p>
                            <p className="text-sm text-gray-500">{activity.time}</p>
                          </div>
                        </div>
                      );
                    })}
                  </div>
                </div>
              </div>

              {/* Recommended Next */}
              <div className="bg-gradient-to-r from-[#6C6CFF]/10 to-[#8a8aff]/10 rounded-2xl p-6 border border-[rgba(108,108,255,0.2)]">
                <div className="flex flex-col md:flex-row items-center gap-6">
                  <div className="p-4 bg-[#6C6CFF]/20 rounded-2xl">
                    <FaRobot className="w-12 h-12 text-[#6C6CFF]" />
                  </div>
                  <div className="flex-1 text-center md:text-left">
                    <h3 className="text-xl font-bold text-white mb-2">Ready for the Next Challenge?</h3>
                    <p className="text-gray-400">
                      Based on your progress, we recommend starting the "URDF for Humanoids" chapter next.
                    </p>
                  </div>
                  <button
                    onClick={() => history.push('/docs/01-module-ros/04-urdf-for-humanoids')}
                    className="flex items-center gap-2 px-6 py-3 bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] text-white rounded-xl font-semibold hover:shadow-lg hover:shadow-[rgba(108,108,255,0.3)] transition-all whitespace-nowrap"
                  >
                    Start Now
                    <FaArrowRight className="w-4 h-4" />
                  </button>
                </div>
              </div>
            </div>
          )}

          {activeTab === 'profile' && (
            <div className="animate-fadeIn">
              <UserProfile user={user} onProfileUpdate={handleProfileUpdate} />
            </div>
          )}

          {activeTab === 'progress' && (
            <div className="space-y-8 animate-fadeIn">
              {/* Progress Overview */}
              <div className="bg-[#0E1330] rounded-2xl p-8 border border-[rgba(255,255,255,0.1)]">
                <h3 className="text-2xl font-bold text-white mb-6">Learning Progress</h3>

                <div className="space-y-6">
                  {[
                    { module: 'Module 1: ROS 2', chapters: 4, completed: 3, color: '#6C6CFF' },
                    { module: 'Module 2: Digital Twin', chapters: 4, completed: 1, color: '#22c55e' },
                    { module: 'Module 3: NVIDIA Isaac', chapters: 4, completed: 0, color: '#f97316' },
                    { module: 'Module 4: VLA Models', chapters: 4, completed: 0, color: '#eab308' },
                  ].map((module, i) => (
                    <div key={i}>
                      <div className="flex items-center justify-between mb-2">
                        <span className="text-white font-medium">{module.module}</span>
                        <span className="text-gray-400 text-sm">{module.completed}/{module.chapters} chapters</span>
                      </div>
                      <div className="w-full h-3 bg-[rgba(255,255,255,0.1)] rounded-full overflow-hidden">
                        <div
                          className="h-full rounded-full transition-all"
                          style={{
                            width: `${(module.completed / module.chapters) * 100}%`,
                            background: module.color
                          }}
                        ></div>
                      </div>
                    </div>
                  ))}
                </div>
              </div>

              {/* Achievements */}
              <div className="bg-[#0E1330] rounded-2xl p-8 border border-[rgba(255,255,255,0.1)]">
                <h3 className="text-2xl font-bold text-white mb-6">Achievements</h3>
                <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                  {[
                    { title: 'First Steps', desc: 'Complete first chapter', earned: true },
                    { title: 'Quick Learner', desc: '5 day streak', earned: true },
                    { title: 'Quiz Master', desc: 'Pass 10 quizzes', earned: true },
                    { title: 'ROS Expert', desc: 'Complete Module 1', earned: false },
                  ].map((achievement, i) => (
                    <div
                      key={i}
                      className={`p-4 rounded-xl border text-center transition-all ${
                        achievement.earned
                          ? 'bg-[rgba(108,108,255,0.1)] border-[rgba(108,108,255,0.3)]'
                          : 'bg-[rgba(255,255,255,0.02)] border-[rgba(255,255,255,0.05)] opacity-50'
                      }`}
                    >
                      <div className={`w-12 h-12 mx-auto mb-3 rounded-full flex items-center justify-center ${
                        achievement.earned ? 'bg-[#6C6CFF]/20' : 'bg-gray-800'
                      }`}>
                        <FaTrophy className={`w-6 h-6 ${achievement.earned ? 'text-[#6C6CFF]' : 'text-gray-600'}`} />
                      </div>
                      <p className={`font-semibold ${achievement.earned ? 'text-white' : 'text-gray-500'}`}>
                        {achievement.title}
                      </p>
                      <p className="text-xs text-gray-500 mt-1">{achievement.desc}</p>
                    </div>
                  ))}
                </div>
              </div>
            </div>
          )}
        </div>
      </main>
    </Layout>
  );
};

export default DashboardPage;