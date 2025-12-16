import React from 'react';
import { motion } from 'framer-motion';

interface IconBubbleProps {
  icon: React.ReactNode;
  size?: 'sm' | 'md' | 'lg';
  color?: 'primary' | 'secondary' | 'accent' | 'neon-purple' | 'neon-pink';
  animated?: boolean;
  pulse?: boolean;
  className?: string;
}

const IconBubble: React.FC<IconBubbleProps> = ({
  icon,
  size = 'md',
  color = 'primary',
  animated = false,
  pulse = false,
  className = '',
}) => {
  const sizeClasses = {
    sm: 'w-10 h-10 text-lg',
    md: 'w-14 h-14 text-xl',
    lg: 'w-20 h-20 text-2xl',
  };

  const colorClasses = {
    primary: 'bg-neon-purple-500/10 text-neon-purple-400 border border-neon-purple-500/30',
    secondary: 'bg-dark-card text-dark-text border border-dark-card/50',
    accent: 'bg-neon-pink-500/10 text-neon-pink-400 border border-neon-pink-500/30',
    'neon-purple': 'bg-neon-purple-500/20 text-neon-purple-300 border border-neon-purple-500/50',
    'neon-pink': 'bg-neon-pink-500/20 text-neon-pink-300 border border-neon-pink-500/50',
  };

  const baseClasses = `flex items-center justify-center rounded-full ${sizeClasses[size]} ${colorClasses[color]} ${className}`;

  const bubbleContent = (
    <div className={baseClasses}>
      {icon}
    </div>
  );

  if (animated) {
    return (
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        whileHover={{ scale: 1.1 }}
        whileTap={{ scale: 0.95 }}
        className={pulse ? "animate-pulse-neon" : ""}
      >
        {bubbleContent}
      </motion.div>
    );
  }

  return bubbleContent;
};

export default IconBubble;