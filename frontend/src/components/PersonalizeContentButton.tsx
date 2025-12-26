import React, { useState, useEffect } from 'react';
import { FaLock, FaUnlock } from 'react-icons/fa';
import { isAuthenticated } from '../utils/session';
import { getPersonalizedContent } from '../services/api';
import '../css/personalization.css'; // Import personalization animations

interface PersonalizeContentButtonProps {
  chapterPath: string;
  onPersonalize: (personalizedContent: string) => void;
  className?: string;
}

const PersonalizeContentButton: React.FC<PersonalizeContentButtonProps> = ({
  chapterPath,
  onPersonalize,
  className = '',
}) => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false); // Track successful personalization
  // Default to `false` for the initial server-side render.
  const [isLoggedIn, setIsLoggedIn] = useState(false);

  // After the component mounts on the client, check the real auth status.
  // This avoids SSR hydration mismatches.
  useEffect(() => {
    setIsLoggedIn(isAuthenticated());
  }, []);

  const handlePersonalizeClick = async () => {
    if (!isLoggedIn) {
      window.location.href = '/login';
      return;
    }

    setIsLoading(true);
    setError(null);
    setSuccess(false); // Reset success state

    try {
      // Using the API service which handles authentication automatically
      // Clean chapterPath to handle Docusaurus pathing patterns
      let cleanedChapterPath = chapterPath;

      // Remove leading slash if present
      if (cleanedChapterPath.startsWith('/')) {
        cleanedChapterPath = cleanedChapterPath.substring(1);
      }

      // Remove 'docs/' prefix if present
      if (cleanedChapterPath.startsWith('docs/')) {
        cleanedChapterPath = cleanedChapterPath.substring(5); // Remove 'docs/'
      }

      // Remove file extensions (.md, .mdx) if present
      cleanedChapterPath = cleanedChapterPath.replace(/\.(md|mdx)$/, '');

      // Additional cleaning: remove any remaining extensions and normalize
      const lastDotIndex = cleanedChapterPath.lastIndexOf('.');
      if (lastDotIndex > cleanedChapterPath.lastIndexOf('/')) {
        // If there's a dot after the last slash, remove everything from that dot onwards
        cleanedChapterPath = cleanedChapterPath.substring(0, lastDotIndex);
      }

      // Log the path for debugging
      console.log('Debug: cleanedChapterPath before API call:', cleanedChapterPath);
      const data = await getPersonalizedContent(cleanedChapterPath);

      if (data.personalized_content) {
        onPersonalize(data.personalized_content);
        setSuccess(true); // Show success indicator

        // Auto-hide success message after 3 seconds
        setTimeout(() => setSuccess(false), 3000);
      } else {
        throw new Error('Personalized content not found in API response.');
      }
    } catch (err: unknown) {
      console.error('Error personalizing content:', err);

      // Type guard to check if err is an axios error
      const axiosError = err as { response?: { status: number }; code?: string; message?: string };

      // Handle auth failures specifically: redirect to login.
      if (axiosError.response?.status === 401 || axiosError.response?.status === 403) {
        window.location.href = '/login';
        return;
      }

      // Specific error message for network errors
      if (axiosError.code === 'ERR_NETWORK') {
        setError('Network error: Unable to connect to the server. Please check if the backend is running.');
      } else {
        setError(axiosError.message || 'An unexpected error occurred. Please try again.');
      }
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`personalize-content-container ${className}`}>
      <button
        onClick={handlePersonalizeClick}
        disabled={isLoading}
        className={`
          personalize-content-button group
          relative overflow-hidden
          flex items-center justify-center gap-2
          px-6 py-3
          font-semibold text-sm uppercase tracking-wide
          text-white
          bg-gradient-to-r from-[#6C6CFF] to-[#8a8aff]
          rounded-full
          border border-transparent
          shadow-[0_4px_15px_rgba(108,108,255,0.3)]
          transition-all duration-300 ease-in-out
          hover:shadow-[0_6px_25px_rgba(108,108,255,0.5)]
          hover:scale-105
          hover:-translate-y-1
          disabled:opacity-50
          disabled:cursor-not-allowed
          disabled:hover:scale-100
          disabled:hover:translate-y-0
          ${isLoading ? 'animate-pulse' : ''}
          ${success ? 'success bg-gradient-to-r from-green-500 to-emerald-500 !shadow-[0_4px_15px_rgba(34,197,94,0.4)]' : ''}
        `}
        aria-label="Personalize Content"
      >
        {/* Animated background glow effect */}
        <span className="absolute inset-0 bg-gradient-to-r from-[#8585ff] to-[#a9a9ff] opacity-0 group-hover:opacity-100 transition-opacity duration-300 rounded-full blur-xl"></span>

        {/* Shimmer effect */}
        <span className="absolute inset-0 overflow-hidden rounded-full">
          <span className="absolute inset-0 bg-gradient-to-r from-transparent via-white to-transparent opacity-20 -translate-x-full group-hover:translate-x-full transition-transform duration-1000 ease-in-out"></span>
        </span>

        {/* Button content */}
        <span className="relative z-10 flex items-center gap-2">
          {isLoading ? (
            <>
              <svg
                className="animate-spin h-5 w-5 text-white"
                xmlns="http://www.w3.org/2000/svg"
                fill="none"
                viewBox="0 0 24 24"
              >
                <circle
                  className="opacity-25"
                  cx="12"
                  cy="12"
                  r="10"
                  stroke="currentColor"
                  strokeWidth="4"
                ></circle>
                <path
                  className="opacity-75"
                  fill="currentColor"
                  d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                ></path>
              </svg>
              <span className="font-bold">Personalizing...</span>
            </>
          ) : success ? (
            <>
              <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
                <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd"/>
              </svg>
              <span className="font-bold">Personalized!</span>
            </>
          ) : (
            <>
              {/* Magical sparkle icon */}
              <svg className="w-5 h-5 transition-transform group-hover:rotate-12 group-hover:scale-110" fill="currentColor" viewBox="0 0 20 20">
                <path d="M9.049 2.927c.3-.921 1.603-.921 1.902 0l1.07 3.292a1 1 0 00.95.69h3.462c.969 0 1.371 1.24.588 1.81l-2.8 2.034a1 1 0 00-.364 1.118l1.07 3.292c.3.921-.755 1.688-1.54 1.118l-2.8-2.034a1 1 0 00-1.175 0l-2.8 2.034c-.784.57-1.838-.197-1.539-1.118l1.07-3.292a1 1 0 00-.364-1.118L2.98 8.72c-.783-.57-.38-1.81.588-1.81h3.461a1 1 0 00.951-.69l1.07-3.292z"/>
              </svg>
              <span className="font-bold">
                {isLoggedIn ? 'Personalize' : 'Login to Personalize'}
              </span>
              {isLoggedIn && <FaUnlock className="ml-1 opacity-75" />}
            </>
          )}
        </span>
      </button>

      {/* Error message */}
      {error && (
        <div className="error-message mt-2 p-2 bg-red-100 border border-red-400 text-red-700 rounded animate-fadeIn">
          {error}
        </div>
      )}

      {/* Success message */}
      {success && (
        <div className="success-message mt-2 p-2 bg-green-100 border border-green-400 text-green-700 rounded flex items-center animate-fadeIn">
          <svg className="w-4 h-4 mr-2" fill="currentColor" viewBox="0 0 20 20">
            <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd"/>
          </svg>
          <span className="text-sm font-medium">✨ Content personalized successfully!</span>
        </div>
      )}
    </div>
  );
};

export default PersonalizeContentButton;