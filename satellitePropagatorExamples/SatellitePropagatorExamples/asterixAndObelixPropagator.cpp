/*    Copyright (c) 2010-2013, Delft University of Technology
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without modification, are
 *    permitted provided that the following conditions are met:
 *      - Redistributions of source code must retain the above copyright notice, this list of
 *        conditions and the following disclaimer.
 *      - Redistributions in binary form must reproduce the above copyright notice, this list of
 *        conditions and the following disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *      - Neither the name of the Delft University of Technology nor the names of its contributors
 *        may be used to endorse or promote products derived from this software without specific
 *        prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 *    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *    OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *    Changelog
 *      YYMMDD    Author            Comment
 *      101111    K. Kumar          File created.
 *      110113    K. Kumar          Scenario updated to use latest version of code; added file
 *                                  header and footer.
 *      110202    K. Kumar          Scenario updated to use latest version of code.
 *      110216    K. Kumar          Migrated to applications namespace.
 *      110217    K. Kumar          Function name changed.
 *      110815    K. Kumar          Updated with mass of Asterix.
 *      111024    K. Kumar          Modified to be executable program with main-function as
 *                                  suggested by M. Persson.
 *      120221    K. Kumar          Rewrote application from scratch; now propagates two
 *                                  satellites.
 *      120502    K. Kumar          Updated code to use shared pointers.
 *      121030    K. Kumar          Updated code to use new state derivative models.
 *      130107    K. Kumar          Updated license in file header.
 *      130225    K. Kumar          Updated gravitational acceleration model references; renamed
 *                                  file; fixed error in assigning Obelix state derivative model;
 *                                  made variables const-correct.
 *
 *    References
 *
 *    Notes
 *
 */

#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <Tudat/Astrodynamics/BasicAstrodynamics/orbitalElementConversions.h>
#include <Tudat/Astrodynamics/BasicAstrodynamics/physicalConstants.h>
#include <Tudat/Astrodynamics/BasicAstrodynamics/unitConversions.h>
#include <Tudat/Mathematics/NumericalIntegrators/rungeKutta4Integrator.h>

#include <Tudat/Astrodynamics/BasicAstrodynamics/accelerationModel.h>
#include <Tudat/Astrodynamics/BasicAstrodynamics/stateVectorIndices.h>
#include <Tudat/Astrodynamics/Gravitation/centralJ2J3J4GravityModel.h>
#include <Tudat/Astrodynamics/StateDerivativeModels/cartesianStateDerivativeModel.h>
#include <Tudat/Astrodynamics/StateDerivativeModels/compositeStateDerivativeModel.h>
#include <Tudat/InputOutput/basicInputOutput.h>
#include <Tudat/Mathematics/BasicMathematics/linearAlgebraTypes.h>

#include <iostream>
#include <fstream>
#include <limits>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "SatellitePropagatorExamples/body.h"

//! Execute propagation of orbits of Asterix and Obelix around the Earth.
int main( )
{
    using namespace satellite_propagator_examples;

    using tudat::basic_astrodynamics::AccelerationModel3dPointer;
    using tudat::orbital_element_conversions::semiMajorAxisIndex;
    using tudat::orbital_element_conversions::eccentricityIndex;
    using tudat::orbital_element_conversions::inclinationIndex;
    using tudat::orbital_element_conversions::argumentOfPeriapsisIndex;
    using tudat::orbital_element_conversions::longitudeOfAscendingNodeIndex;
    using tudat::orbital_element_conversions::trueAnomalyIndex;
    using tudat::orbital_element_conversions::xCartesianPositionIndex;
    using tudat::orbital_element_conversions::yCartesianPositionIndex;
    using tudat::orbital_element_conversions::zCartesianPositionIndex;
    using tudat::orbital_element_conversions::xCartesianVelocityIndex;
    using tudat::orbital_element_conversions::yCartesianVelocityIndex;
    using tudat::orbital_element_conversions::zCartesianVelocityIndex;

    using tudat::basic_mathematics::Vector6d;

    using tudat::gravitation::CentralJ2J3J4GravitationalAccelerationModel;
    using tudat::gravitation::CentralJ2J3J4GravitationalAccelerationModelPointer;

    using tudat::input_output::DoubleKeyTypeVectorXdValueTypeMap;
    using tudat::input_output::writeDataMapToTextFile;

    using tudat::numerical_integrators::RungeKutta4Integrator;

    using tudat::orbital_element_conversions::convertKeplerianToCartesianElements;

    using tudat::state_derivative_models::CartesianStateDerivativeModel6d;
    using tudat::state_derivative_models::CartesianStateDerivativeModel6dPointer;
    using tudat::state_derivative_models::CompositeStateDerivativeModel;

    using tudat::unit_conversions::convertDegreesToRadians;

    typedef Eigen::Matrix< double, 12, 1 > Vector12d;
    typedef CompositeStateDerivativeModel< double, Vector12d, Vector6d >
            CompositeStateDerivativeModel12d;

    ///////////////////////////////////////////////////////////////////////////

    // Input deck.

    // Set output directory.
    const std::string outputDirectory = "";
    if( outputDirectory == "" )
    {
        std::cerr<<"Error, output directory not specified (modify outputDirectory variable to "<<
                   " required output directory)."<<std::endl;
    }

    // Set simulation start epoch.
    const double simulationStartEpoch = 0.0;

    // Set simulation end epoch.
    const double simulationEndEpoch = tudat::physical_constants::JULIAN_DAY;

    // Set numerical integration fixed step size.
    const double fixedStepSize = 60.0;

    // Set initial conditions for satellites that will be propagated in this simulation.
    // The initial conditions are given in Keplerian elements and later on converted to
    // Cartesian elements.

    // Set Keplerian elements for Asterix.
    Vector6d asterixInitialStateInKeplerianElements;
    asterixInitialStateInKeplerianElements( semiMajorAxisIndex ) = 7500.0e3;
    asterixInitialStateInKeplerianElements( eccentricityIndex ) = 0.1;
    asterixInitialStateInKeplerianElements( inclinationIndex ) = convertDegreesToRadians( 85.3 );
    asterixInitialStateInKeplerianElements( argumentOfPeriapsisIndex )
            = convertDegreesToRadians( 235.7 );
    asterixInitialStateInKeplerianElements( longitudeOfAscendingNodeIndex )
            = convertDegreesToRadians( 23.4 );
    asterixInitialStateInKeplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 139.87 );

    // Set Keplerian elements for Obelix.
    Vector6d obelixInitialStateInKeplerianElements( 6 );
    obelixInitialStateInKeplerianElements( semiMajorAxisIndex ) = 12040.6e3;
    obelixInitialStateInKeplerianElements( eccentricityIndex ) = 0.4;
    obelixInitialStateInKeplerianElements( inclinationIndex ) = convertDegreesToRadians( -23.5 );
    obelixInitialStateInKeplerianElements( argumentOfPeriapsisIndex )
            = convertDegreesToRadians( 10.6 );
    obelixInitialStateInKeplerianElements( longitudeOfAscendingNodeIndex )
            = convertDegreesToRadians( 367.9 );
    obelixInitialStateInKeplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 93.4 );

    // Set Earth gravitational parameter [m^3 s^-2].
    const double earthGravitationalParameter = 3.986004415e14;

    // Set spherical harmonics zonal term coefficients.
    const double earthJ2 = 0.0010826269;
    const double earthJ3 = -0.0000025323;
    const double earthJ4 = -0.0000016204;

    // Set equatorial radius of Earth [m].
    const double earthEquatorialRadius = 6378.1363e3;

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////

    // Convert initial states from Keplerian to Cartesian elements.

    // Convert Asterix state from Keplerian elements to Cartesian elements.
    const Vector6d asterixInitialState = convertKeplerianToCartesianElements(
                asterixInitialStateInKeplerianElements,
                earthGravitationalParameter );

    // Convert Obelix state from Keplerian elements to Cartesian elements.
    const Vector6d obelixInitialState = convertKeplerianToCartesianElements(
                obelixInitialStateInKeplerianElements,
                earthGravitationalParameter );

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////

    // Create Asterix and Obelix satellites, and gravitational acceleration models.

    // Create Asterix and set initial state and epoch.
    const BodyPointer asterix = boost::make_shared< Body >( asterixInitialState );

    // Create gravitational acceleration model for asterix.
    const CentralJ2J3J4GravitationalAccelerationModelPointer asterixGravityModel
            = boost::make_shared< CentralJ2J3J4GravitationalAccelerationModel >(
                boost::bind( &Body::getCurrentPosition, asterix ),
                earthGravitationalParameter, earthEquatorialRadius, earthJ2, earthJ3, earthJ4  );

    // Create Cartesian state derivative model for asterix.
    const CartesianStateDerivativeModel6d::AccelerationModelPointerVector asterixGravity
            = boost::assign::list_of( asterixGravityModel );

    // Create Obelix and set initial state and epoch.
    const BodyPointer obelix = boost::make_shared< Body >( obelixInitialState );

    // Create gravitational acceleration model for obelix.
    const CentralJ2J3J4GravitationalAccelerationModelPointer obelixGravityModel
            = boost::make_shared< CentralJ2J3J4GravitationalAccelerationModel >(
                boost::bind( &Body::getCurrentPosition, obelix ),
                earthGravitationalParameter, earthEquatorialRadius, earthJ2, earthJ3, earthJ4  );

    // Create Cartesian state derivative model model for obelix.
   const  CartesianStateDerivativeModel6d::AccelerationModelPointerVector obelixGravity
            = boost::assign::list_of( obelixGravityModel );

    // Add Asterix and Obelix to list of satellites.
    ListOfSatellites satellites;
    satellites[ asterix ] = asterixGravity;
    satellites[ obelix ] = obelixGravity;

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////

    // Create Cartesian state derivative models for Asterix and Obelix.

    // Create Cartesian state derivative model for Asterix.
    const CartesianStateDerivativeModel6dPointer asterixStateDerivativeModel
            = boost::make_shared< CartesianStateDerivativeModel6d >(
                asterixGravity, boost::bind( &Body::setCurrentTimeAndState, asterix, _1, _2 ) );

    // Create Cartesian state derivative model for Obelix.
    const CartesianStateDerivativeModel6dPointer obelixStateDerivativeModel
            = boost::make_shared< CartesianStateDerivativeModel6d >(
                obelixGravity, boost::bind( &Body::setCurrentTimeAndState, obelix, _1, _2 ) );

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////

    // Construct composite state derivative model for Asterix and Obelix.

    // Create state derivative model map and bind Asterix and Obelix, in that order.
    CompositeStateDerivativeModel12d::VectorStateDerivativeModelMap stateDerivativeModelMap;

    stateDerivativeModelMap[ std::make_pair( 0, 6 ) ]
            = boost::bind( &CartesianStateDerivativeModel6d::computeStateDerivative,
                           asterixStateDerivativeModel, _1, _2 );

    stateDerivativeModelMap[ std::make_pair( 6, 6 ) ]
            = boost::bind( &CartesianStateDerivativeModel6d::computeStateDerivative,
                           obelixStateDerivativeModel, _1, _2 );

    // Create data updater.
    DataUpdater updater( satellites );

    // Create composite state derivative model.
    boost::shared_ptr< CompositeStateDerivativeModel12d > stateDerivativeModel
            = boost::make_shared< CompositeStateDerivativeModel12d >(
                stateDerivativeModelMap,
                boost::bind( &DataUpdater::updateBodyData, updater, _1, _2 ) );

    ///////////////////////////////////////////////////////////////////////////

    // Set up numerical integrator and execute simulation.

    // Declare Runge-Kutta 4 integrator.
    // Since the state derivative function is a member-function, it must be passed by using
    // boost::bind. The "_1" and "_2" in the boost::bind call specifies that the argument list
    // for the computeStateDerivative function takes two arguments (t, x).
    RungeKutta4Integrator< double, Vector12d, Vector12d > rungeKutta4(
                boost::bind( &CompositeStateDerivativeModel12d::computeStateDerivative,
                             stateDerivativeModel, _1, _2 ),
                0.0, ( Eigen::VectorXd( 12 ) << asterixInitialState,
                       obelixInitialState ).finished( ) );

    // Set running time, updated after each step that the numerical integrator takes.
    double runningTime = simulationStartEpoch;

    // Declare propagation history to store state history of satellites.
    DoubleKeyTypeVectorXdValueTypeMap asterixPropagationHistory;
    DoubleKeyTypeVectorXdValueTypeMap obelixPropagationHistory;

    // Set initial states in propagation history.
    asterixPropagationHistory[ 0.0 ] = asterixInitialState;
    obelixPropagationHistory[ 0.0 ] = obelixInitialState;

    // Execute simulation from start to end epoch and save intermediate states in propagation
    // history.
    while ( runningTime < simulationEndEpoch )
    {
        // Execute integration step and store state at end.
        Vector12d integratedState = rungeKutta4.performIntegrationStep( fixedStepSize );

        // Update running time to value of current time.
        runningTime = rungeKutta4.getCurrentIndependentVariable( );

        // Disassemble state into states per body, and store in propagation history.
        asterixPropagationHistory[ runningTime ] = integratedState.segment( 0, 6 );
        obelixPropagationHistory[ runningTime ] = integratedState.segment( 6, 6 );
    }

    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////

    // Write results to file.

    // Write Asterix propagation history to file.
    writeDataMapToTextFile( asterixPropagationHistory,
                            "asterixPropagationHistory.dat",
                            outputDirectory,
                            "",
                            std::numeric_limits< double >::digits10,
                            std::numeric_limits< double >::digits10,
                            "," );

    // Write obelix propagation history to file.
    writeDataMapToTextFile( obelixPropagationHistory,
                            "obelixPropagationHistory.dat",
                            outputDirectory,
                            "",
                            std::numeric_limits< double >::digits10,
                            std::numeric_limits< double >::digits10,
                            "," );

    ///////////////////////////////////////////////////////////////////////////

    return 0;
}
