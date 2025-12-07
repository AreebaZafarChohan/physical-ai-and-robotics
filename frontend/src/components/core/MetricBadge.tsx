import React from 'react';

interface MetricBadgeProps {
  value: string | number;
  label: string;
  icon?: React.ReactNode;
  variant?: 'primary' | 'secondary' | 'accent';
  size?: 'sm' | 'md' | 'lg';
}

const MetricBadge: React.FC<MetricBadgeProps> = ({
  value,
  label,
  icon,
  variant = 'primary',
  size = 'md',
}) => {
  const variantClasses = {
    primary: 'bg-neon-purple-500/10 text-neon-purple-300 border border-neon-purple-500/30',
    secondary: 'bg-dark-card text-dark-text border border-dark-card/50',
    accent: 'bg-neon-pink-500/10 text-neon-pink-300 border border-neon-pink-500/30',
  };

  const sizeClasses = {
    sm: 'p-2 text-xs',
    md: 'p-3 text-sm',
    lg: 'p-4 text-base',
  };

  return (
    <div className={`flex flex-col items-center justify-center rounded-lg ${variantClasses[variant]} ${sizeClasses[size]} min-w-[80px]`}>
      <div className="flex items-center justify-center space-x-1">
        {icon && <span className="text-lg">{icon}</span>}
        <span className="font-bold">{value}</span>
      </div>
      <span className="text-xs opacity-80 mt-1">{label}</span>
    </div>
  );
};

export default MetricBadge;