import React from 'react';
import { motion } from 'framer-motion';
import IconBubble from './IconBubble';

interface FeatureCardProps {
  title: string;
  description: string;
  icon?: React.ReactNode;
  iconColor?: 'primary' | 'secondary' | 'accent' | 'neon-purple' | 'neon-pink';
  badge?: string;
  animated?: boolean;
  onClick?: () => void;
}

const FeatureCard: React.FC<FeatureCardProps> = ({
  title,
  description,
  icon,
  iconColor = 'primary',
  badge,
  animated = true,
  onClick,
}) => {
  const cardContent = (
    <div 
      className={`bg-surface rounded-xl p-6 border border-transparent hover:border-neon-purple-500/30 transition-all duration-300 shadow-lg hover:shadow-xl hover:shadow-neon-purple/10 ${
        onClick ? 'cursor-pointer' : ''
      }`}
      onClick={onClick}
    >
      <div className="flex flex-col h-full">
        <div className="flex items-start justify-between mb-4">
          {icon && <IconBubble icon={icon} size="sm" color={iconColor} animated={false} />}
          {badge && (
            <span className="bg-neon-purple-500/20 text-neon-purple-300 text-xs px-2 py-1 rounded-full">
              {badge}
            </span>
          )}
        </div>
        <h3 className="text-xl font-bold text-white mb-2">{title}</h3>
        <p className="text-white/80 mb-4 flex-grow">{description}</p>
      </div>
    </div>
  );

  if (animated) {
    return (
      <motion.div
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

export default FeatureCard;