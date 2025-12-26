/**
 * Content Parser utility for handling personalization markers in content
 */

/**
 * Interface for personalization marker
 */
export interface PersonalizationMarker {
  type: string;
  content: string;
  startIndex: number;
  endIndex: number;
}

/**
 * Parse content to find personalization markers
 * @param content - The content string to parse
 * @returns Array of PersonalizationMarker objects found in the content
 */
export const findPersonalizationMarkers = (content: string): PersonalizationMarker[] => {
  const markers: PersonalizationMarker[] = [];
  
  // Regular expression to match personalization markers
  // Pattern: {{personalize:markerType}}content{{personalize:end}}
  const markerRegex = /{{personalize:([^}]+)}}([\s\S]*?){{personalize:end}}/g;
  
  let match;
  while ((match = markerRegex.exec(content)) !== null) {
    const fullMatch = match[0];
    const markerType = match[1];
    const markerContent = match[2];
    const startIndex = match.index;
    const endIndex = match.index + fullMatch.length;
    
    markers.push({
      type: markerType,
      content: markerContent,
      startIndex,
      endIndex
    });
  }
  
  return markers;
};

/**
 * Replace personalization markers in content based on user profile
 * @param content - Original content with markers
 * @param userProfile - User's profile information
 * @returns Content with appropriate personalization markers replaced
 */
export const replacePersonalizationMarkers = (
  content: string, 
  userProfile: {
    software_background: string[];
    hardware_background: string[];
    experience_level: string;
  }
): string => {
  // Find all personalization markers
  const markers = findPersonalizationMarkers(content);
  
  // Sort markers in reverse order by position so replacements don't affect indices
  const sortedMarkers = markers.sort((a, b) => b.startIndex - a.startIndex);
  
  let result = content;
  
  for (const marker of sortedMarkers) {
    // Check if this marker should be shown based on user profile
    if (shouldShowMarker(marker.type, userProfile)) {
      // Replace the entire marker with its content
      result = result.substring(0, marker.startIndex) + 
               marker.content + 
               result.substring(marker.endIndex);
    } else {
      // Remove the entire marker (don't show content)
      result = result.substring(0, marker.startIndex) + 
               result.substring(marker.endIndex);
    }
  }
  
  return result;
};

/**
 * Determine if a marker should be shown based on user profile
 * @param markerType - Type of the personalization marker
 * @param userProfile - User's profile information
 * @returns Boolean indicating if marker should be shown
 */
const shouldShowMarker = (
  markerType: string, 
  userProfile: {
    software_background: string[];
    hardware_background: string[];
    experience_level: string;
  }
): boolean => {
  // Special case for experience level markers
  if (markerType === userProfile.experience_level) {
    return true;
  }
  
  // Check if markerType matches any of the user's software backgrounds
  if (userProfile.software_background.some(sb => 
    markerType === sb.toLowerCase().replace(/\s+/g, '-') || 
    markerType === sb.toLowerCase().replace(/\s+/g, '_')
  )) {
    return true;
  }
  
  // Check if markerType matches any of the user's hardware backgrounds
  if (userProfile.hardware_background.some(hb => 
    markerType === hb.toLowerCase().replace(/\s+/g, '-') || 
    markerType === hb.toLowerCase().replace(/\s+/g, '_')
  )) {
    return true;
  }
  
  // If the marker type is a combination (e.g., "python-raspberry-pi"), check if all parts match
  if (markerType.includes('-')) {
    const parts = markerType.split('-');
    const userBgCombined = [
      ...userProfile.software_background.map(sb => sb.toLowerCase().replace(/\s+/g, '-')),
      ...userProfile.hardware_background.map(hb => hb.toLowerCase().replace(/\s+/g, '-'))
    ];
    
    return parts.every(part => 
      userBgCombined.some(ubg => ubg.includes(part) || part.includes(ubg))
    );
  }
  
  // If no specific match found, don't show the content
  return false;
};

/**
 * Extract all unique marker types from content
 * @param content - The content string to analyze
 * @returns Array of unique marker types found in the content
 */
export const extractMarkerTypes = (content: string): string[] => {
  const markers = findPersonalizationMarkers(content);
  const uniqueTypes = new Set(markers.map(marker => marker.type));
  return Array.from(uniqueTypes);
};

/**
 * Remove all personalization markers from content, keeping only the generic content
 * @param content - The content string with markers
 * @returns Content with all personalization markers removed
 */
export const removeMarkers = (content: string): string => {
  const markers = findPersonalizationMarkers(content);
  
  // Sort markers in reverse order by position so replacements don't affect indices
  const sortedMarkers = markers.sort((a, b) => b.startIndex - a.startIndex);
  
  let result = content;
  
  for (const marker of sortedMarkers) {
    // Remove the entire marker
    result = result.substring(0, marker.startIndex) + 
             result.substring(marker.endIndex);
  }
  
  return result;
};