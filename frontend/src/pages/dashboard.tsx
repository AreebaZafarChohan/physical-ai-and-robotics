import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import UserProfile from '../components/UserProfile';
import { getCurrentUser } from '../services/auth';
import { useHistory } from '@docusaurus/router';

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
        <main className="flex items-center justify-center py-12 px-4 sm:px-6 lg:px-8">
          <p>Loading user data...</p>
        </main>
      </Layout>
    );
  }

  if (error) {
    return (
      <Layout title="Dashboard" description="User Dashboard">
        <main className="flex items-center justify-center py-12 px-4 sm:px-6 lg:px-8">
          <p className="text-red-500">Error: {error}</p>
        </main>
      </Layout>
    );
  }

  // Fallback if user is somehow null after loading and no error (should be redirected by now)
  if (!user) {
    return (
      <Layout title="Dashboard" description="User Dashboard">
        <main className="flex items-center justify-center py-12 px-4 sm:px-6 lg:px-8">
          <p>You need to be logged in to view this page.</p>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title="Dashboard" description="User Dashboard">
      <main className="py-12 px-4 sm:px-6 lg:px-8">
        <div className="max-w-4xl mx-auto space-y-8">
          <h2 className="mt-6 text-center text-3xl font-extrabold">
            Welcome, {user.username}!
          </h2>
          <UserProfile user={user} />
        </div>
      </main>
    </Layout>
  );
};

export default DashboardPage;
