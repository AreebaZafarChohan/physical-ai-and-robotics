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
            {user?.username || 'Account'}
          </a>
          <ul className="dropdown__menu">
            <li>
              <a
                className="dropdown__link"
                onClick={() => history.push(`${baseUrl}dashboard`)}
                style={{ cursor: 'pointer' }}
              >
                Dashboard
              </a>
            </li>
            <li>
              <a
                className="dropdown__link"
                onClick={handleLogout}
                style={{ cursor: 'pointer' }}
              >
                Logout
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
        Login
      </a>
      <a
        className="navbar__item navbar__link"
        onClick={() => history.push(`${baseUrl}register`)}
        style={{ cursor: 'pointer' }}
      >
        Register
      </a>
    </>
  );
};

export default AuthNavbarItem;