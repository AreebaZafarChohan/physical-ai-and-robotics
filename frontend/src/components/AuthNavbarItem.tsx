import React, { useEffect, useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { isLoggedIn, logoutUser, getCurrentUser, User } from '../services/auth';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// This is a placeholder for a real dropdown component
const Dropdown: React.FC<{
  children: React.ReactNode;
  label: React.ReactNode;
  className?: string;
}> = ({ children, label, className }) => {
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = React.useRef<HTMLDivElement>(null);

  const toggleDropdown = () => setIsOpen(!isOpen);

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };
    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  return (
    <div className={`dropdown dropdown--hoverable ${className}`} ref={dropdownRef}>
      <a className="navbar__item navbar__link" onClick={toggleDropdown}>
        {label}
      </a>
      {isOpen && (
        <ul className="dropdown__menu" style={{ display: 'block' }}>
          {children}
        </ul>
      )}
    </div>
  );
};


const AuthNavbarItem: React.FC = () => {
  const history = useHistory();
  const location = useLocation();
  const {
    siteConfig: { baseUrl },
  } = useDocusaurusContext();
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
    // Re-check auth status when location changes (e.g., after login/logout redirect)
  }, [location]);

  const handleLogout = async () => {
    await logoutUser();
    setLoggedIn(false);
    setUser(null);
    history.push('/login');
  };

  if (loggedIn) {
    return (
      <Dropdown
        label={
          <div className="flex items-center space-x-2">
            <div className="w-8 h-8 rounded-full bg-primary flex items-center justify-center text-white font-bold">
              {user?.username ? user.username.charAt(0).toUpperCase() : 'U'}
            </div>
            <span>{user?.username || 'Account'}</span>
          </div>
        }
        className="navbar__item navbar__link--active"
      >
        <li>
          <a
            className="dropdown__link"
            onClick={() => history.push(`${baseUrl}dashboard`)}
          >
            Dashboard
          </a>
        </li>
        <li>
          <a className="dropdown__link" onClick={handleLogout}>
            Logout
          </a>
        </li>
      </Dropdown>
    );
  }

  return (
    <>
      <a
        className="navbar__item navbar__link"
        onClick={() => history.push(`${baseUrl}login`)}
      >
        Login
      </a>
      <a
        className="navbar__item navbar__link"
        onClick={() => history.push(`${baseUrl}register`)}
      >
        Register
      </a>
    </>
  );
};

export default AuthNavbarItem;
