import React from 'react';

interface GridWrapperProps {
  children: React.ReactNode;
  cols?: 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 12;
  gap?: 'xs' | 'sm' | 'md' | 'lg' | 'xl' | '2xl';
  className?: string;
  responsive?: boolean; // Whether to adjust cols based on screen size
}

const GridWrapper: React.FC<GridWrapperProps> = ({
  children,
  cols = 3,
  gap = 'md',
  className = '',
  responsive = true,
}) => {
  // Map gap sizes to Tailwind classes
  const gapClasses = {
    xs: 'gap-2',
    sm: 'gap-3',
    md: 'gap-4',
    lg: 'gap-6',
    xl: 'gap-8',
    '2xl': 'gap-12',
  };

  // Default grid classes with responsive behavior
  const baseClasses = responsive 
    ? `grid grid-cols-1 sm:grid-cols-2 md:grid-cols-${cols} lg:grid-cols-${cols}`
    : `grid grid-cols-${cols}`;

  const gridClasses = `${baseClasses} ${gapClasses[gap]} ${className}`;

  return (
    <div className={gridClasses}>
      {children}
    </div>
  );
};

export default GridWrapper;