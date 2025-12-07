import React from 'react';
import { motion } from 'framer-motion';
import IconBubble from './IconBubble';

type SpectrumType = 'assisted' | 'driven' | 'native';

interface SpectrumCardProps {
  type: SpectrumType;
  title: string;
  description: string;
  icon?: React.ReactNode;
  animated?: boolean;
  onClick?: () => void;
}

const SpectrumCard: React.FC<SpectrumCardProps> = ({
  type,
  title,
  description,
  icon,
  animated = true,
  onClick,
}) => {
  // Define type-specific styles
  const typeStyles = {
    assisted: {
      bgColor: 'bg-neon-purple-500/10',
      borderColor: 'border-neon-purple-500/30',
      textColor: 'text-neon-purple-300',
      iconColor: 'neon-purple' as const,
      gradient: 'from-neon-purple-500/20 to-transparent',
    },
    driven: {
      bgColor: 'bg-gradient-to-br from-neon-purple-500/10 to-neon-pink-500/10',
      borderColor: 'border-gradient-to-r from-neon-purple-500 to-neon-pink-500',
      textColor: 'text-neon-pink-300',
      iconColor: 'neon-pink' as const,
      gradient: 'from-neon-purple-500/20 via-transparent to-neon-pink-500/20',
    },
    native: {
      bgColor: 'bg-neon-pink-500/10',
      borderColor: 'border-neon-pink-500/30',
      textColor: 'text-neon-pink-300',
      iconColor: 'neon-pink' as const,
      gradient: 'from-neon-pink-500/20 to-transparent',
    },
  };

  const styles = typeStyles[type];

  const cardContent = (
    <div 
      className={`${styles.bgColor} rounded-xl p-6 border ${styles.borderColor} transition-all duration-300 shadow-lg hover:shadow-xl group ${
        onClick ? 'cursor-pointer' : ''
      } relative overflow-hidden`}
      onClick={onClick}
    >
      <div className="absolute inset-0 bg-gradient-to-r bg-size-200 bg-pos-0 group-hover:bg-pos-100 bg-gradient-to-r from-transparent via-white/5 to-transparent transition-all duration-700"></div>
      <div className="relative z-10 flex flex-col h-full">
        <div className="mb-4">
          {icon && <IconBubble icon={icon} size="md" color={styles.iconColor} animated={false} />}
        </div>
        <h3 className="text-2xl font-bold text-light-text mb-3">{title}</h3>
        <p className="text-light-text/80 mb-4 flex-grow">{description}</p>
        <div className="mt-auto pt-4 border-t border-dark-card/50">
          <span className={`inline-block px-3 py-1 rounded-full text-sm font-medium ${styles.textColor}`}>
            AI {type.charAt(0).toUpperCase() + type.slice(1)}
          </span>
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

export default SpectrumCard;