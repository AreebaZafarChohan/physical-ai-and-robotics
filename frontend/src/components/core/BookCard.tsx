import React from 'react';
import { motion } from 'framer-motion';

interface BookCardProps {
  title: string;
  author: string;
  description: string;
  coverImage?: string; // URL to the book cover image
  chapters?: number;
  lessons?: number;
  exercises?: number;
  animated?: boolean;
  onClick?: () => void;
}

const BookCard: React.FC<BookCardProps> = ({
  title,
  author,
  description,
  coverImage,
  chapters,
  lessons,
  exercises,
  animated = true,
  onClick,
}) => {
  const cardContent = (
    <div 
      className={`bg-surface rounded-xl overflow-hidden border border-transparent hover:border-neon-purple-500/30 transition-all duration-300 shadow-lg hover:shadow-xl hover:shadow-neon-purple/10 ${
        onClick ? 'cursor-pointer' : ''
      }`}
      onClick={onClick}
    >
      <div className="p-6">
        <div className="flex items-start mb-4">
          {coverImage ? (
            <div className="w-16 h-20 bg-gradient-to-br from-neon-purple-500/20 to-neon-pink-500/20 rounded shadow-lg flex items-center justify-center mr-4">
              <img src={coverImage} alt={title} className="w-full h-full object-cover rounded" />
            </div>
          ) : (
            <div className="w-16 h-20 bg-gradient-to-br from-neon-purple-500/20 to-neon-pink-500/20 rounded shadow-lg flex items-center justify-center mr-4">
              <span className="text-2xl">📖</span>
            </div>
          )}
          <div>
            <h3 className="text-xl font-bold text-white">{title}</h3>
            <p className="text-white/70 text-sm">by {author}</p>
          </div>
        </div>
        <p className="text-white/80 text-sm mb-4">{description}</p>
        
        {/* Metrics section */}
        <div className="flex justify-between text-sm">
          {chapters !== undefined && (
            <div className="text-center">
              <div className="font-bold text-neon-purple-400">{chapters}</div>
              <div className="text-white/60">Chapters</div>
            </div>
          )}
          {lessons !== undefined && (
            <div className="text-center">
              <div className="font-bold text-neon-purple-400">{lessons}</div>
              <div className="text-white/60">Lessons</div>
            </div>
          )}
          {exercises !== undefined && (
            <div className="text-center">
              <div className="font-bold text-neon-purple-400">{exercises}</div>
              <div className="text-white/60">Exercises</div>
            </div>
          )}
        </div>
      </div>
    </div>
  );

  if (animated) {
    return (
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        whileHover={{ y: -10, boxShadow: '0 25px 50px -12px rgba(101, 101, 255, 0.25)' }}
        whileTap={{ scale: 0.98 }}
        transition={{ type: "spring", stiffness: 300, damping: 20 }}
      >
        {cardContent}
      </motion.div>
    );
  }

  return cardContent;
};

export default BookCard;