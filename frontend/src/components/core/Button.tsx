import React from 'react';
import { motion } from 'framer-motion';
import type { ButtonHTMLAttributes, DetailedHTMLProps } from 'react';

type ButtonVariant = 'primary' | 'secondary' | 'outline';
type ButtonSize = 'sm' | 'md' | 'lg';

interface ButtonProps extends DetailedHTMLProps<ButtonHTMLAttributes<HTMLButtonElement>, HTMLButtonElement> {
  variant?: ButtonVariant;
  size?: ButtonSize;
  fullWidth?: boolean;
  children: React.ReactNode;
  icon?: React.ReactNode;
  iconPosition?: 'left' | 'right';
}

const Button: React.FC<ButtonProps> = ({
  variant = 'primary',
  size = 'md',
  fullWidth = false,
  children,
  icon,
  iconPosition = 'left',
  className = '',
  onClick,
  disabled,
  ...props
}) => {
  const baseClasses = [
    'inline-flex items-center justify-center',
    'font-medium rounded-lg',
    'transition-all duration-300 ease-in-out',
    'focus:outline-none focus:ring-2 focus:ring-offset-2',
    'disabled:opacity-50 disabled:cursor-not-allowed',
    'active:scale-[0.98]',
  ].join(' ');

  const variantClasses = {
    primary: [
      'bg-neon-purple-500 text-white',
      'hover:bg-neon-pink-500 hover:shadow-neon-pink',
      'focus:ring-neon-purple-300 focus:ring-opacity-50',
    ],
    secondary: [
      'bg-dark-card text-dark-text',
      'border border-neon-purple-500',
      'hover:bg-neon-purple-500 hover:text-white',
      'focus:ring-neon-purple-300 focus:ring-opacity-50',
    ],
    outline: [
      'bg-transparent text-neon-purple-500',
      'border border-neon-purple-500',
      'hover:bg-neon-purple-500 hover:text-white',
      'focus:ring-neon-purple-300 focus:ring-opacity-50',
    ],
  };

  const sizeClasses = {
    sm: ['px-3', 'py-1.5', 'text-sm'],
    md: ['px-4', 'py-2', 'text-base'],
    lg: ['px-6', 'py-3', 'text-lg'],
  };

  const fullClasses = fullWidth ? 'w-full' : '';

  const classes = [
    baseClasses,
    ...variantClasses[variant],
    ...sizeClasses[size],
    fullClasses,
    className,
  ].join(' ');

  const content = (
    <button
      className={classes}
      onClick={onClick}
      disabled={disabled}
      {...props}
    >
      {icon && iconPosition === 'left' && <span className="mr-2">{icon}</span>}
      {children}
      {icon && iconPosition === 'right' && <span className="ml-2">{icon}</span>}
    </button>
  );

  return (
    <motion.div whileHover={{ scale: !disabled ? 1.02 : 1 }} whileTap={{ scale: !disabled ? 0.98 : 1 }}>
      {content}
    </motion.div>
  );
};

export default Button;