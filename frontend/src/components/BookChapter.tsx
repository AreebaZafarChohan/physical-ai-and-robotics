import React from 'react';
import { usePersonalization } from '../hooks/usePersonalization';

interface BookChapterProps {
  chapterTitle: string;
  chapterContent: string;
}

const BookChapter: React.FC<BookChapterProps> = ({ chapterTitle, chapterContent }) => {
  const { personalizationData, isLoading, error } = usePersonalization();

  return (
    <div className="container mx-auto px-4 py-8">
      <h1 className="text-4xl font-bold mb-6">{chapterTitle}</h1>
      <div className="prose lg:prose-xl max-w-none mb-8">
        {chapterContent}
      </div>

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
    </div>
  );
};

export default BookChapter;
