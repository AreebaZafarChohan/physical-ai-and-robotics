from typing import List, Optional
from sqlmodel import Session, select
from backend.src.models.personalization_rules import PersonalizationRule, PersonalizationRuleCreate, PersonalizationRuleUpdate


class PersonalizationRuleRepository:
    """
    Repository class for personalization rule database operations
    """
    
    def __init__(self, session: Session):
        self.session = session
    
    def get_rule_by_id(self, rule_id: int) -> Optional[PersonalizationRule]:
        """
        Get a personalization rule by its ID
        """
        return self.session.get(PersonalizationRule, rule_id)
    
    def get_active_rules_by_path(self, content_path: str) -> List[PersonalizationRule]:
        """
        Get all active personalization rules for a specific content path
        """
        statement = select(PersonalizationRule).where(
            (PersonalizationRule.content_path == content_path) &
            (PersonalizationRule.is_active == True)
        )
        return self.session.exec(statement).all()
    
    def get_all_active_rules(self) -> List[PersonalizationRule]:
        """
        Get all active personalization rules
        """
        statement = select(PersonalizationRule).where(PersonalizationRule.is_active == True)
        return self.session.exec(statement).all()
    
    def create_rule(self, rule_data: PersonalizationRuleCreate) -> PersonalizationRule:
        """
        Create a new personalization rule
        """
        rule = PersonalizationRule.from_orm(rule_data) if hasattr(PersonalizationRule, 'from_orm') else PersonalizationRule(
            content_path=rule_data.content_path,
            user_profile_criteria=rule_data.user_profile_criteria,
            content_variants=rule_data.content_variants,
            description=rule_data.description,
            priority=rule_data.priority,
            is_active=rule_data.is_active
        )
        
        self.session.add(rule)
        self.session.commit()
        self.session.refresh(rule)
        return rule
    
    def update_rule(self, rule_id: int, rule_data: PersonalizationRuleUpdate) -> Optional[PersonalizationRule]:
        """
        Update an existing personalization rule
        """
        rule = self.session.get(PersonalizationRule, rule_id)
        if not rule:
            return None
        
        # Update the rule with new data
        update_data = rule_data.dict(exclude_unset=True)
        for field, value in update_data.items():
            setattr(rule, field, value)
        
        self.session.add(rule)
        self.session.commit()
        self.session.refresh(rule)
        return rule
    
    def delete_rule(self, rule_id: int) -> bool:
        """
        Delete a personalization rule
        """
        rule = self.session.get(PersonalizationRule, rule_id)
        if not rule:
            return False
        
        self.session.delete(rule)
        self.session.commit()
        return True
    
    def get_rules_by_criteria(self, profile_criteria: dict) -> List[PersonalizationRule]:
        """
        Get personalization rules that match specific profile criteria
        """
        # This is a simplified implementation - in a real scenario, we might need more
        # sophisticated matching based on the criteria
        all_rules = self.get_all_active_rules()
        
        matching_rules = []
        for rule in all_rules:
            # Check if the rule's criteria match the provided profile criteria
            matches = True
            for key, value in profile_criteria.items():
                if key in rule.user_profile_criteria:
                    rule_criteria_value = rule.user_profile_criteria[key]
                    # If the rule's criteria value is a list, check if there's an intersection
                    if isinstance(rule_criteria_value, list) and isinstance(value, list):
                        if not set(rule_criteria_value).intersection(set(value)):
                            matches = False
                            break
                    elif rule_criteria_value != value:
                        matches = False
                        break
            
            if matches:
                matching_rules.append(rule)
        
        return matching_rules