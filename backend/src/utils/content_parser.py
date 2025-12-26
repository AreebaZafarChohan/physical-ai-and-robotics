import re
from typing import List, Dict, Any


def parse_content_with_markers(content: str) -> Dict[str, Any]:
    """
    Parse content and extract information about personalization markers
    """
    result = {
        'content_without_markers': content,
        'markers_found': [],
        'marker_count': 0
    }
    
    # Regular expression to match personalization markers
    # Pattern: {{personalize:marker_type}}content{{personalize:end}}
    marker_pattern = r'\{\{personalize:([^}]+)\}\}([\s\S]*?)\{\{personalize:end\}\}'
    
    matches = re.findall(marker_pattern, content)
    result['marker_count'] = len(matches)
    
    # Store information about each marker
    for marker_type, marker_content in matches:
        result['markers_found'].append({
            'type': marker_type,
            'content': marker_content.strip()
        })
    
    return result


def replace_markers_in_content(content: str, user_profile: Dict[str, Any]) -> str:
    """
    Replace personalization markers in content based on user profile
    """
    # Regular expression to match personalization markers
    marker_pattern = r'\{\{personalize:([^}]+)\}\}([\s\S]*?)\{\{personalize:end\}\}'
    
    def replace_marker(match):
        marker_type = match.group(1)
        marker_content = match.group(2)
        
        # Check if this marker should be shown based on user profile
        if should_show_marker(marker_type, user_profile):
            return marker_content
        else:
            return ''  # Remove the content if marker shouldn't be shown
    
    # Replace all markers in the content
    result_content = re.sub(marker_pattern, replace_marker, content)
    
    return result_content


def should_show_marker(marker_type: str, user_profile: Dict[str, Any]) -> bool:
    """
    Determine if a marker should be shown based on user profile
    """
    # Check experience level
    if marker_type == user_profile.get('experience_level', ''):
        return True

    # Handle experience level ranges (e.g., if user is 'intermediate', show 'beginner' content too)
    experience_level = user_profile.get('experience_level', '').lower()
    if experience_level == 'intermediate' and marker_type == 'beginner':
        return True
    if experience_level == 'advanced' and (marker_type == 'beginner' or marker_type == 'intermediate'):
        return True

    # Check software background
    software_background = user_profile.get('software_background', [])
    for sb in software_background:
        # Normalize the software name to match marker types (e.g., 'Python' -> 'python', 'python-focused')
        normalized_sb = sb.lower().replace(' ', '-').replace('_', '-')
        if marker_type in normalized_sb or normalized_sb in marker_type:
            return True
        # Check for partial matches
        if marker_type.replace('-', '').replace('_', '') in normalized_sb.replace('-', '').replace('_', ''):
            return True

    # Check hardware background
    hardware_background = user_profile.get('hardware_background', [])
    for hb in hardware_background:
        # Normalize the hardware name to match marker types
        normalized_hb = hb.lower().replace(' ', '-').replace('_', '-')
        if marker_type in normalized_hb or normalized_hb in marker_type:
            return True
        # Check for partial matches
        if marker_type.replace('-', '').replace('_', '') in normalized_hb.replace('-', '').replace('_', ''):
            return True

    # Special case: marker type might be a combination like "python-raspberry-pi"
    if '-' in marker_type:
        parts = marker_type.split('-')
        user_backgrounds = [
            sb.lower().replace(' ', '-').replace('_', '-')
            for sb in software_background + hardware_background
        ]

        # Check if all parts of the marker are covered by user's background
        all_parts_match = True
        for part in parts:
            if not any(part in bg or bg in part for bg in user_backgrounds):
                all_parts_match = False
                break
        if all_parts_match:
            return True

    # Default: don't show marker content
    return False


def adapt_content_by_user_profile(content: str, user_profile: Dict[str, Any]) -> str:
    """
    Adapt content based on user profile using multiple strategies
    """
    # First, sanitize the content for security
    content = sanitize_content(content)

    # Then, handle experience level adaptations
    content = _adapt_by_experience_level(content, user_profile.get('experience_level', 'beginner'))

    # Then, apply marker-based personalization
    content = replace_markers_in_content(content, user_profile)

    # Validate content structure after adaptation
    if not validate_content_structure(content):
        raise ValueError("Content structure is invalid after adaptation")

    return content


def _adapt_by_experience_level(content: str, experience_level: str) -> str:
    """
    Adapt content based on experience level
    """
    if experience_level.lower() == 'beginner':
        # For beginners, add more explanations
        content = content.replace('<p>', '<p class="beginner-focused">', content.count('<p>'))
    elif experience_level.lower() == 'advanced':
        # For advanced users, potentially add more complex examples
        pass  # In a full implementation, we would have more logic here

    return content


def extract_marker_types(content: str) -> List[str]:
    """
    Extract all unique marker types from content
    """
    marker_pattern = r'\{\{personalize:([^}]+)\}\}'
    matches = re.findall(marker_pattern, content)
    return list(set(matches))  # Return unique marker types


def sanitize_content(content: str) -> str:
    """
    Sanitize content to prevent XSS and other security issues
    """
    # This is a more comprehensive implementation
    # For production use, consider using a dedicated library like bleach

    # Remove script tags and their content
    content = re.sub(r'<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>', '', content, flags=re.IGNORECASE | re.DOTALL)

    # Remove style tags with js expressions
    content = re.sub(r'<style[^>]*>.*?<\/style>', '', content, flags=re.IGNORECASE | re.DOTALL)

    # Remove event handlers (like onclick, onload, etc.)
    content = re.sub(r'\s*on\w+\s*=\s*["\'][^"\']*["\']', '', content, flags=re.IGNORECASE)

    # Remove javascript: and vbscript: URIs
    content = re.sub(r'(?i)(javascript:|vbscript:|data:)', '', content)

    # Remove href/src attributes with javascript
    content = re.sub(r'(?i)(href|src)\s*=\s*["\']\s*javascript:[^"\']*["\']', '', content)

    # Remove iframe tags
    content = re.sub(r'<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>', '', content, flags=re.IGNORECASE | re.DOTALL)

    # Remove object and embed tags
    content = re.sub(r'<(object|embed)\b[^<]*(?:(?!<\/\1>)<[^<]*)*<\/\1>', '', content, flags=re.IGNORECASE | re.DOTALL)

    return content


def validate_content_structure(content: str) -> bool:
    """
    Validate that content markers are properly structured
    """
    import re

    # Find all markers in order
    all_markers = []
    for match in re.finditer(r'\{\{personalize:[^}]+\}\}|{{personalize:end}}', content):
        all_markers.append(match.group())

    # Count and verify proper ordering using a stack-like approach
    depth = 0
    for marker in all_markers:
        if marker.startswith('{{personalize:') and not marker == '{{personalize:end}}':
            # It's a start marker
            depth += 1
        elif marker == '{{personalize:end}}':
            # It's an end marker
            depth -= 1
            if depth < 0:
                return False  # More end markers than start markers at this point

    # All markers should be properly closed
    return depth == 0