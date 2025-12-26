import React, { useState, useEffect } from 'react';
import { usePersonalization } from '../hooks/usePersonalization';
import PersonalizeContentButton from './PersonalizeContentButton'; // Import the button component
import { isAuthenticated } from '../utils/session'; // Import the session utility
import '../css/personalization.css'; // Import personalization styles

interface BookChapterProps {
  chapterTitle: string;
  chapterContent: string;
  chapterPath?: string; // Optional specific path for personalization API
}

const BookChapter: React.FC<BookChapterProps> = ({
  chapterTitle,
  chapterContent,
  chapterPath: propChapterPath
}) => {
  const { personalizationData, isLoading, error } = usePersonalization();
  const [displayedContent, setDisplayedContent] = useState(chapterContent);
  const [currentPath, setCurrentPath] = useState<string>('');
  const [isPersonalized, setIsPersonalized] = useState(false); // Track if content is personalized

  // Get the current path for the personalization API if not provided via props
  useEffect(() => {
    if (!propChapterPath && typeof window !== 'undefined') {
      // Extract path from URL - remove leading slash and any query parameters
      const path = window.location.pathname.split('?')[0];
      setCurrentPath(path);
    } else if (propChapterPath) {
      setCurrentPath(propChapterPath);
    }
  }, [propChapterPath]);

  const handlePersonalize = (personalizedContent: string) => {
    setDisplayedContent(personalizedContent);
    setIsPersonalized(true); // Mark as personalized for visual indicator
  };

  // Only show personalization data if the user is authenticated
  const isUserAuthenticated = isAuthenticated();

  // Use the provided path or the detected path for the button
  const actualChapterPath = propChapterPath || currentPath || chapterTitle;

  return (
    <div className="container mx-auto px-4 py-8">
      <div className="flex justify-end mb-4">
        <PersonalizeContentButton
          chapterPath={actualChapterPath}
          onPersonalize={handlePersonalize}
        />
      </div>

      {/* Show personalization indicator banner */}
      {isPersonalized && (
        <div className="mb-4 p-4 bg-gradient-to-r from-blue-50 to-purple-50 border-l-4 border-blue-500 rounded-r-lg shadow-sm animate-fadeIn">
          <div className="flex items-center">
            <svg className="w-5 h-5 text-blue-500 mr-2" fill="currentColor" viewBox="0 0 20 20">
              <path d="M10 2a8 8 0 100 16 8 8 0 000-16zM9 9a1 1 0 012 0v4a1 1 0 11-2 0V9zm1-5a1 1 0 100 2 1 1 0 000-2z"/>
            </svg>
            <span className="text-sm font-medium text-blue-800">
              ✨ Content personalized based on your profile
            </span>
          </div>
        </div>
      )}

      <h1 className="text-4xl font-bold mb-6">{chapterTitle}</h1>

      {/* Content container with personalization styling */}
      <div className={`
        prose lg:prose-xl max-w-none mb-8
        transition-all duration-500 ease-in-out
        ${isPersonalized ? 'personalized-content border-l-4 border-blue-300 pl-6 bg-gradient-to-r from-blue-50/30 to-transparent rounded-lg py-4' : ''}
      `}>
        {displayedContent}
      </div>

      {isUserAuthenticated && (
        <>
          {isLoading && (
            <div className="mt-8 p-4 bg-blue-100 border border-blue-400 text-blue-700 rounded-md">
              Loading personalization data...
            </div>
          )}

          {error && (
            <div className="mt-8 p-4 bg-red-100 border border-red-400 text-red-700 rounded-md">
              Error loading personalization: {error}
            </div>
          )}

          {personalizationData && (
            <div className="mt-8 p-4 bg-green-100 border border-green-400 text-green-700 rounded-md">
              <h2 className="text-2xl font-semibold mb-3">Personalized Suggestions:</h2>
              {personalizationData.software_background.length > 0 && (
                <p className="mb-2">
                  Based on your software background ({personalizationData.software_background.join(', ')}),
                  you might find these advanced topics relevant: [Dynamic content related to software background].
                </p>
              )}
              {personalizationData.hardware_background.length > 0 && (
                <p>
                  Considering your hardware background ({personalizationData.hardware_background.join(', ')}),
                  we recommend exploring these hands-on projects: [Dynamic content related to hardware background].
                </p>
              )}
              {personalizationData.software_background.length === 0 && personalizationData.hardware_background.length === 0 && (
                <p>No specific background information found to provide personalized suggestions.</p>
              )}
            </div>
          )}
        </>
      )}
    </div>
  );
};

export default BookChapter;
