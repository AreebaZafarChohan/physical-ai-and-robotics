import React, { useEffect, useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { isLoggedIn, logoutUser, getCurrentUser, User } from '../services/auth';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const AuthNavbarItem: React.FC = () => {
  const history = useHistory();
  const location = useLocation();
  const { siteConfig: { baseUrl } } = useDocusaurusContext();
  const [loggedIn, setLoggedIn] = useState(false);
  const [user, setUser] = useState<User | null>(null);

  useEffect(() => {
    const checkAuthStatus = async () => {
      const status = isLoggedIn();
      setLoggedIn(status);
      if (status) {
        const currentUser = await getCurrentUser();
        setUser(currentUser);
      } else {
        setUser(null);
      }
    };

    checkAuthStatus();
  }, [location]);

  const handleLogout = async () => {
    await logoutUser();
    setLoggedIn(false);
    setUser(null);
    history.push('/login');
  };

  // Logged in user - use Docusaurus dropdown
  if (loggedIn) {
    return (
      <>
        <div className="navbar__item dropdown dropdown--hoverable dropdown--right">
          <a className="navbar__link" style={{ cursor: 'pointer' }}>
            <span className="flex items-center gap-2">
              <div className="w-8 h-8 rounded-full bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff] flex items-center justify-center text-sm font-bold">
                {user?.username?.charAt(0)?.toUpperCase() || 'U'}
              </div>
              <span>{user?.username || 'Account'}</span>
            </span>
          </a>
          <ul className="dropdown__menu">
            <li>
              <a
                className="dropdown__link"
                onClick={() => history.push(`${baseUrl}dashboard`)}
                style={{ cursor: 'pointer' }}
              >
                <span className="flex items-center gap-2">
                  <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <rect x="3" y="3" width="7" height="9" rx="1"></rect>
                    <rect x="14" y="3" width="7" height="5" rx="1"></rect>
                    <rect x="14" y="12" width="7" height="9" rx="1"></rect>
                    <rect x="3" y="16" width="7" height="5" rx="1"></rect>
                  </svg>
                  Dashboard
                </span>
              </a>
            </li>
            <li>
              <a
                className="dropdown__link"
                onClick={handleLogout}
                style={{ cursor: 'pointer' }}
              >
                <span className="flex items-center gap-2">
                  <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4"></path>
                    <polyline points="16 17 21 12 16 7"></polyline>
                    <line x1="21" y1="12" x2="9" y2="12"></line>
                  </svg>
                  Logout
                </span>
              </a>
            </li>
          </ul>
        </div>
      </>
    );
  }

  // Not logged in
  return (
    <>
      <a
        className="navbar__item navbar__link"
        onClick={() => history.push(`${baseUrl}login`)}
        style={{ cursor: 'pointer' }}
      >
        <span className="flex items-center gap-1">
          <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M15 3h4a2 2 0 0 1 2 2v14a2 2 0 0 1-2 2h-4"></path>
            <polyline points="10 17 15 12 10 7"></polyline>
            <line x1="15" y1="12" x2="3" y2="12"></line>
          </svg>
          Login
        </span>
      </a>
      <a
        className="navbar__item navbar__link"
        onClick={() => history.push(`${baseUrl}register`)}
        style={{ cursor: 'pointer' }}
      >
        <span className="flex items-center gap-1">
          <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M16 21v-2a4 4 0 0 0-4-4H6a4 4 0 0 0-4 4v2"></path>
            <circle cx="9" cy="7" r="4"></circle>
            <path d="M22 21v-2a4 4 0 0 0-3-3.87"></path>
            <path d="M16 3.13a4 4 0 0 1 0 7.75"></path>
          </svg>
          Register
        </span>
      </a>
    </>
  );
};

export default AuthNavbarItem;