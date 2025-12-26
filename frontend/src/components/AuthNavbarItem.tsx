import React, { useEffect, useState, useRef } from 'react';
import { useHistory } from '@docusaurus/router';
import { isLoggedIn, logoutUser, getCurrentUser, User } from '../services/auth';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { FaUser, FaSignOutAlt, FaTachometerAlt, FaSignInAlt, FaUserPlus } from 'react-icons/fa';

// Correct Dropdown component for desktop
const Dropdown: React.FC<{
  children: React.ReactNode;
  label: React.ReactNode;
  className?: string;
}> = ({ children, label, className }) => {
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  const toggleDropdown = (e: React.MouseEvent) => {
    e.preventDefault();
    setIsOpen(!isOpen);
  };

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
    <div className={`relative ${className}`} ref={dropdownRef}>
      <a
        className="navbar__item navbar__link cursor-pointer flex items-center gap-2"
        onClick={toggleDropdown}
      >
        {label}
      </a>
      {isOpen && (
        <ul
          className="dropdown-menu absolute right-0 mt-2 py-2 w-56 rounded-xl shadow-2xl z-50 border overflow-hidden"
          style={{
            display: 'block',
            background: 'linear-gradient(180deg, rgba(14, 19, 48, 0.98) 0%, rgba(21, 26, 61, 0.98) 100%)',
            borderColor: 'rgba(108, 108, 255, 0.3)',
            backdropFilter: 'blur(20px)'
          }}
        >
          {children}
        </ul>
      )}
    </div>
  );
};

const AuthNavbarItem: React.FC = () => {
  const history = useHistory();
  const location = useLocation();
  const { siteConfig: { baseUrl } } = useDocusaurusContext();
  const [loggedIn, setLoggedIn] = useState(false);
  const [user, setUser] = useState<User | null>(null);
  const [isMobile, setIsMobile] = useState(false);

  useEffect(() => {
    // Check if mobile
    const checkMobile = () => {
      setIsMobile(window.innerWidth <= 996);
    };
    checkMobile();
    window.addEventListener('resize', checkMobile);
    return () => window.removeEventListener('resize', checkMobile);
  }, []);

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

  // Logged in user UI
  if (loggedIn) {
    return (
      <div className="auth-navbar-item">
        <Dropdown
          label={
            <div className="flex items-center gap-2">
              <div
                className="w-8 h-8 rounded-full flex items-center justify-center text-white font-bold text-sm"
                style={{
                  background: 'linear-gradient(135deg, #6C6CFF 0%, #8a8aff 100%)',
                  boxShadow: '0 2px 8px rgba(108, 108, 255, 0.4)'
                }}
              >
                {user?.username ? user.username.charAt(0).toUpperCase() : 'U'}
              </div>
              <span className="hidden md:inline font-medium">{user?.username || 'Account'}</span>
              <svg
                className="w-4 h-4 opacity-60"
                fill="none"
                stroke="currentColor"
                viewBox="0 0 24 24"
              >
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
              </svg>
            </div>
          }
        >
          {/* User Info Header */}
          <li className="px-4 py-3 border-b border-[rgba(255,255,255,0.1)]">
            <div className="flex items-center gap-3">
              <div
                className="w-10 h-10 rounded-full flex items-center justify-center text-white font-bold"
                style={{
                  background: 'linear-gradient(135deg, #6C6CFF 0%, #8a8aff 100%)',
                }}
              >
                {user?.username ? user.username.charAt(0).toUpperCase() : 'U'}
              </div>
              <div>
                <p className="text-white font-semibold text-sm">{user?.username}</p>
                <p className="text-gray-400 text-xs truncate max-w-[140px]">{user?.email}</p>
              </div>
            </div>
          </li>

          {/* Dashboard Link */}
          <li>
            <a
              className="flex items-center gap-3 px-4 py-3 text-gray-200 hover:text-white hover:bg-[rgba(108,108,255,0.15)] cursor-pointer transition-all"
              onClick={() => history.push(`${baseUrl}dashboard`)}
            >
              <FaTachometerAlt className="w-4 h-4 text-[#6C6CFF]" />
              <span className="font-medium">Dashboard</span>
            </a>
          </li>

          {/* Profile Link */}
          <li>
            <a
              className="flex items-center gap-3 px-4 py-3 text-gray-200 hover:text-white hover:bg-[rgba(108,108,255,0.15)] cursor-pointer transition-all"
              onClick={() => history.push(`${baseUrl}dashboard`)}
            >
              <FaUser className="w-4 h-4 text-[#6C6CFF]" />
              <span className="font-medium">My Profile</span>
            </a>
          </li>

          {/* Logout */}
          <li className="border-t border-[rgba(255,255,255,0.1)] mt-1">
            <a
              className="flex items-center gap-3 px-4 py-3 text-red-400 hover:text-red-300 hover:bg-[rgba(255,100,100,0.1)] cursor-pointer transition-all"
              onClick={handleLogout}
            >
              <FaSignOutAlt className="w-4 h-4" />
              <span className="font-medium">Logout</span>
            </a>
          </li>
        </Dropdown>
      </div>
    );
  }

  // Not logged in - show Login/Register buttons
  return (
    <div className="auth-navbar-item flex items-center gap-2">
      {/* Login Button */}
      <a
        className="navbar__item navbar__link cursor-pointer flex items-center gap-2 px-4 py-2 rounded-full transition-all hover:bg-[rgba(108,108,255,0.1)]"
        onClick={() => history.push(`${baseUrl}login`)}
        style={{ color: '#F0F0FF' }}
      >
        <FaSignInAlt className="w-4 h-4" />
        <span className="font-medium">Login</span>
      </a>

      {/* Register Button - Highlighted */}
      <a
        className="navbar__item navbar__link cursor-pointer flex items-center gap-2 px-5 py-2 rounded-full font-semibold transition-all"
        onClick={() => history.push(`${baseUrl}register`)}
        style={{
          background: 'linear-gradient(135deg, #6C6CFF 0%, #8a8aff 100%)',
          color: 'white',
          boxShadow: '0 2px 10px rgba(108, 108, 255, 0.3)',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.transform = 'translateY(-2px)';
          e.currentTarget.style.boxShadow = '0 4px 15px rgba(108, 108, 255, 0.5)';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'translateY(0)';
          e.currentTarget.style.boxShadow = '0 2px 10px rgba(108, 108, 255, 0.3)';
        }}
      >
        <FaUserPlus className="w-4 h-4" />
        <span>Register</span>
      </a>
    </div>
  );
};

export default AuthNavbarItem;
