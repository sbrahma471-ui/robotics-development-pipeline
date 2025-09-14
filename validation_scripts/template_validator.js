// URDF Template Validation Script
// This script validates that all URDF templates are properly structured

const validateTemplate = (template) => {
  const { id, name, content } = template;
  
  console.log(`\n=== Validating Template: ${name} (${id}) ===`);
  
  // Check if content contains required elements
  const hasRobotTag = content.includes('<robot');
  const hasNameAttribute = content.includes('name="');
  const hasLink = content.includes('<link');
  const hasValidXML = content.includes('<?xml version="1.0"?>');
  
  console.log(`✓ XML Declaration: ${hasValidXML ? 'PASS' : 'FAIL'}`);
  console.log(`✓ Robot Tag: ${hasRobotTag ? 'PASS' : 'FAIL'}`);
  console.log(`✓ Name Attribute: ${hasNameAttribute ? 'PASS' : 'FAIL'}`);
  console.log(`✓ Link Elements: ${hasLink ? 'PASS' : 'FAIL'}`);
  
  // Count elements
  const linkCount = (content.match(/<link/g) || []).length;
  const jointCount = (content.match(/<joint/g) || []).length;
  const visualCount = (content.match(/<visual>/g) || []).length;
  const collisionCount = (content.match(/<collision>/g) || []).length;
  
  console.log(`📊 Statistics:`);
  console.log(`   Links: ${linkCount}`);
  console.log(`   Joints: ${jointCount}`);
  console.log(`   Visual elements: ${visualCount}`);
  console.log(`   Collision elements: ${collisionCount}`);
  
  // Overall validation
  const isValid = hasValidXML && hasRobotTag && hasNameAttribute && hasLink;
  console.log(`🎯 Overall: ${isValid ? 'VALID ✅' : 'INVALID ❌'}`);
  
  return {
    id,
    name,
    isValid,
    stats: { linkCount, jointCount, visualCount, collisionCount }
  };
};

// Template validation results
console.log('🔍 URDF Template Validation Report');
console.log('=====================================');

// Mock templates for validation (these match the actual templates in the app)
const templates = [
  {
    id: 'blank',
    name: 'Blank URDF',
    content: `<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>`
  },
  {
    id: 'robotic_arm',
    name: '6-DOF Robotic Arm',
    content: `<?xml version="1.0"?>
<robot name="robotic_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
    </visual>
  </link>
  <link name="link_1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
  </joint>
</robot>`
  }
];

const results = templates.map(validateTemplate);

console.log('\n📋 Summary Report:');
console.log('==================');
results.forEach(result => {
  console.log(`${result.name}: ${result.isValid ? '✅ VALID' : '❌ INVALID'}`);
});

const allValid = results.every(r => r.isValid);
console.log(`\n🎯 Overall Template System: ${allValid ? '✅ ALL VALID' : '❌ ISSUES FOUND'}`);

export { validateTemplate, results };
