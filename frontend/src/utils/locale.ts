import { useLocation } from '@docusaurus/router';

// Define supported locales
export const SUPPORTED_LOCALES = ['en', 'ur', 'ar'] as const;
export type Locale = typeof SUPPORTED_LOCALES[number];

/**
 * Gets the current locale from the URL path
 * @returns The current locale or the default 'en' if none is detected
 */
export const getCurrentLocale = (): Locale => {
  if (typeof window !== 'undefined') {
    const pathSegments = window.location.pathname.split('/');
    const potentialLocale = pathSegments[1];
    
    if (SUPPORTED_LOCALES.includes(potentialLocale as any)) {
      return potentialLocale as Locale;
    }
  }
  
  // Default to English if no locale is detected in the path
  return 'en';
};

/**
 * Hook to get the current locale from the router location
 * @returns The current locale
 */
export const useCurrentLocale = (): Locale => {
  const location = useLocation();
  const pathSegments = location.pathname.split('/');
  const potentialLocale = pathSegments[1];
  
  if (SUPPORTED_LOCALES.includes(potentialLocale as any)) {
    return potentialLocale as Locale;
  }
  
  return 'en'; // Default to English
};

/**
 * Checks if the current locale is right-to-left
 * @param locale The locale to check
 * @returns True if the locale is RTL, false otherwise
 */
export const isRTL = (locale: Locale = getCurrentLocale()): boolean => {
  return locale === 'ur' || locale === 'ar';
};

/**
 * Gets the direction attribute value for a given locale
 * @param locale The locale to check
 * @returns 'rtl' if the locale is right-to-left, 'ltr' otherwise
 */
export const getDirection = (locale: Locale = getCurrentLocale()): 'ltr' | 'rtl' => {
  return isRTL(locale) ? 'rtl' : 'ltr';
};

/**
 * Gets the base URL for a given locale
 * @param locale The target locale
 * @returns The base URL for the locale
 */
export const getLocaleBaseUrl = (locale: Locale): string => {
  if (locale === 'en') {
    // For English (default locale), we don't include the locale in the URL
    return '/';
  }
  return `/${locale}/`;
};

/**
 * Translates a path to a specific locale
 * @param path The original path
 * @param targetLocale The target locale
 * @returns The path with the locale prefix
 */
export const translatePath = (path: string, targetLocale: Locale): string => {
  // Remove existing locale prefix if present
  const pathSegments = path.split('/');
  if (SUPPORTED_LOCALES.includes(pathSegments[1] as any)) {
    // Remove the locale segment
    pathSegments.splice(1, 1);
  }
  
  const newPath = pathSegments.join('/');
  
  if (targetLocale === 'en') {
    // For English (default locale), don't add a prefix
    return newPath;
  }
  
  // Add the target locale prefix
  return `/${targetLocale}${newPath}`;
};

/**
 * Determines if a given locale is valid and supported
 * @param locale The locale to validate
 * @returns True if the locale is supported, false otherwise
 */
export const isValidLocale = (locale: string): locale is Locale => {
  return SUPPORTED_LOCALES.includes(locale as any);
};

/**
 * Gets the direction class name for CSS styling
 * @param locale The locale to check
 * @returns The appropriate CSS class name for direction
 */
export const getDirectionClassName = (locale: Locale = getCurrentLocale()): string => {
  return isRTL(locale) ? 'rtl' : 'ltr';
};