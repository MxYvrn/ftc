# How to Run Stress Tests

## Prerequisites
- Android Studio or command line with Gradle
- Java JDK 8 or higher
- FTC SDK dependencies (automatically downloaded by Gradle)

## Running Tests

### Command Line (Gradle)

```bash
# Run all tests
./gradlew :TeamCode:test

# Run only stress tests
./gradlew :TeamCode:test --tests "com.teamcode.stress.*"

# Run specific test class
./gradlew :TeamCode:test --tests "com.teamcode.stress.ConstantsValidationTest"

# Run with verbose output
./gradlew :TeamCode:test --info

# Run tests in continuous mode (auto-rerun on changes)
./gradlew :TeamCode:test --continuous
```

### Android Studio

1. Open project in Android Studio
2. Navigate to `TeamCode/src/test/java/com/teamcode/stress/`
3. Right-click on test file or package
4. Select "Run Tests" or "Debug Tests"

### Test Results

Test results are saved to:
- `TeamCode/build/test-results/testDebugUnitTest/`
- HTML reports: `TeamCode/build/reports/tests/testDebugUnitTest/`

## Expected Test Results

After fixes:
- ✅ `ConstantsValidationTest.testRPMToTPSConversionCorrectness()` - PASS
- ✅ `ConstantsValidationTest.testRPMToTPSWithActualValues()` - PASS
- ✅ `OdometryStressTest.testDivisionByZeroEdgeCases()` - PASS
- ✅ `DriveSubsystemStressTest.testVoltageCompensationBounds()` - PASS
- ✅ `DriveSubsystemStressTest.testVoltageScaleClamping()` - PASS

## Known Issues (Pre-Fix)

Before applying bug fixes, these tests will fail:
- ❌ `ConstantsValidationTest.testRPMToTPSConversionCorrectness()` - FAIL (BUG-001)
- ❌ `ShooterSubsystemStressTest.testVelocitySignConsistency()` - FAIL (BUG-002)

## Continuous Integration

Add to your CI pipeline:
```yaml
# Example GitHub Actions
- name: Run stress tests
  run: ./gradlew :TeamCode:test --tests "com.teamcode.stress.*"
```

## Debugging Failed Tests

1. Check test output for specific assertion failures
2. Review `BUG_LIST.md` for known issues
3. Run individual test methods to isolate problems
4. Use Android Studio debugger to step through code

## Adding New Tests

1. Create test class in `TeamCode/src/test/java/com/teamcode/stress/`
2. Follow naming convention: `*StressTest.java`
3. Use JUnit 4 annotations: `@Test`, `@Before`, `@After`
4. Add test case to `BUG_LIST.md` if testing a bug fix
5. Ensure test is deterministic (no random/fuzzy results)

