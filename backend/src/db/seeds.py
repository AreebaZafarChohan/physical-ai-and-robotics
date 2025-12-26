from sqlmodel import Session
from backend.src.models.personalization_rules import PersonalizationRule
from datetime import datetime
import json


def create_basic_personalization_rules(session: Session):
    """
    Create basic personalization rules for testing
    """
    # Rule 1: Python-specific content
    rule1 = PersonalizationRule(
        content_path="/module-1/chapter-1",
        user_profile_criteria={
            "software_background": ["Python"]
        },
        content_variants={
            "python-focused": "<p>This section includes Python-specific examples and explanations.</p>"
        },
        description="Python-specific content for Chapter 1",
        priority=10,
        is_active=True,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    
    # Rule 2: Beginner-friendly content
    rule2 = PersonalizationRule(
        content_path="/module-1/chapter-1",
        user_profile_criteria={
            "experience_level": ["beginner"]
        },
        content_variants={
            "beginner": "<p>As a beginner, we'll go through this step by step with simple explanations.</p>"
        },
        description="Beginner-friendly content for Chapter 1",
        priority=5,
        is_active=True,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    
    # Rule 3: Raspberry Pi hardware-specific content
    rule3 = PersonalizationRule(
        content_path="/module-2/chapter-1",
        user_profile_criteria={
            "hardware_background": ["Raspberry Pi"]
        },
        content_variants={
            "raspberry-pi": "<p>This section shows how to implement this on a Raspberry Pi.</p>"
        },
        description="Raspberry Pi-specific content for Chapter 2",
        priority=10,
        is_active=True,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    
    # Add all rules to session
    session.add(rule1)
    session.add(rule2)
    session.add(rule3)
    
    # Commit the changes
    session.commit()
    
    print("Basic personalization rules created successfully.")