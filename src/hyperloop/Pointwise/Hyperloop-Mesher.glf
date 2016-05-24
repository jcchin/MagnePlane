#
# Copyright 2015 © Pointwise, Inc.
# All rights reserved.
#
# This sample script is not supported by Pointwise, Inc.
# It is provided freely for demonstration purposes only.
# SEE THE WARRANTY DISCLAIMER AT THE BOTTOM OF THIS FILE.
#

#
# =====================================================================
# GENERATE AN UNSTRUCTURED VOLUME MESH FOR A HYPERLOOP VEHICLE
# =====================================================================
# Originally written by: Zach Davis, Pointwise, Inc.
# Extensive Modification by Christopher Heath, NASA GRC
#
# This script generates an inviscid mesh of a hyperloop
# vehicle traveling in a closed tube environment. The resulting 
# grid is saved to the ../AFLR3 directory. 
#

# --------------------------------------------------------
# -- INITIALIZATION
# --
# -- Load Glyph package, initialize Pointwise, and
# -- define the working directory.
# --
# --------------------------------------------------------

# Load Glyph and Tcl Data Structure Libraries
package require PWI_Glyph

# Initialize Pointwise
pw::Application reset
pw::Application clearModified

# Define Working Directory
set scriptDir [file dirname [info script]]

# --------------------------------------------------------
# -- USER-DEFINED PARAMETERS
# --
# -- Define a set of user-controllable settings.
# --
# --------------------------------------------------------

set fileName            "Hyperloop.pw";  # Aircraft Pointwise filename
set avgDs1                        0.05;  # Initial surface triangle average edge length
set avgDs2                        0.35;  # fuselage IML/OML surface triangle average edge length
set inletGrowthRate                1.2;  # Root/Tip connector distribution growth rate
set nozzleGrowthRate               1.2;  # Root/Tip connector distribution growth rate
set inletSpacing                0.0025;  # Inlet 2D TREX spacing
set nozzleSpacing               0.0025;  # Nozzle 2D TREX spacing
set leteGrowthRate                 1.2;  # Leading/Trailing edge connector distribution layers
set domLayers                       15;  # Layers for 2D T-Rex surface meshing
set domGrowthRate                  1.2;  # Growth rate for 2D T-Rex surface meshing
set aspectRatio                   15.0;  # Aspect ratio for 2D T-Rex surface meshing
set growthRate                     1.2;  # Growth rate for boundary layer extrusion
set boundaryDecay                  0.8;  # Volumetric boundary decay
set numLayers                      100;  # Max number of layers to extrude
set fullLayers                       1;  # Full layers (0 for multi-normals, 1 for single normal)
set collisionBuffer                  2;  # Collision buffer for colliding fronts
set maxAngle                     165.0;  # Max included angle for boundary elements
set centroidSkew                   1.0;  # Max centroid skew for boundary layer elements
set LESpacing                   0.0025;  # Spacing for chord cons to be consistent with TREX
set TESpacing                   0.0025;  # Spacing for chord cons TEs to be consistent with TREX
set EFSpacing                     0.01;  # Spacing for fuselage face connectors            
# --------------------------------------------------------
# -- PROCEDURES
# --
# -- Define procedures for frequent tasks. 
# --
# --------------------------------------------------------

# Get Domains from Underlying Quilt
proc DomFromQuilt { quilt } {
    set gridsOnQuilt [$quilt getGridEntities]

    foreach grid $gridsOnQuilt {
        if { [$grid isOfType pw::DomainUnstructured] } {
            lappend doms $grid
        }
    }

    if { ![info exists doms] } {
        puts "INFO: There are no domains on [$quilt getName]"
        return
    }

    return $doms
}

# Get All Connectors in a Domain
proc ConsFromDom { dom } {
    set numEdges [$dom getEdgeCount]

    for { set i 1 } { $i <= $numEdges } { incr i } {
        set edge [$dom getEdge $i]
        set numCons [$edge getConnectorCount]

        for { set j 1 } { $j <= $numCons } {incr j} {
            lappend cons [$edge getConnector $j]
        }
    }

    return $cons
}

# Apply a Growth Distribution to Selected Connectors
proc RedistCons { rate beginlayers endlayers begin end conList } {
    set conMode [pw::Application begin Modify $conList]
        foreach con $conList {
            $con replaceDistribution 1 [pw::DistributionGrowth create]
            set dist [$con getDistribution 1]
            puts $dist
            $dist setBeginSpacing $begin
            $dist setBeginMode    LayersandRate
            $dist setBeginRate    $rate
            $dist setBeginLayers  $beginlayers

            $dist setEndSpacing $end
            $dist setEndMode    LayersandRate
            $dist setEndRate    $rate
            $dist setEndLayers  $endlayers

            $con setDimensionFromDistribution
        }
    $conMode end
    unset conMode
}

# Computes the Set Containing the Intersection of Set1 & Set2
proc intersect { set1 set2 } {
    set set3 [list]

    foreach item $set1 {
        if { [lsearch -exact $set2 $item] >= 0 } {
            lappend set3 $item
        }
    }

    return $set3
}

# Query the System Clock
proc timestamp {} {
    puts [clock format [clock seconds] -format "%a %b %d %Y %l:%M:%S%p %Z"]
}

# Query the System Clock (ISO 8601 formatting)
proc timestamp_iso {} {
    puts [clock format [clock seconds] -format "%G-%m-%dT%T%Z"]
}

# Convert Time in Seconds to h:m:s Format
proc convSeconds { time } {
    set h [expr { int(floor($time/3600)) }]
    set m [expr { int(floor($time/60)) % 60 }]
    set s [expr { int(floor($time)) % 60 }]
    return [format  "%02d Hours %02d Minutes %02d Seconds" $h $m $s]
}

# --------------------------------------------------------
# -- MAIN ROUTINE
# --
# -- Main meshing procedure:
# --   Load Pointwise File
# --   Apply User Settings
# --   Gather Analysis Model Information
# --   Mesh Surfaces
# --   Refine Wing and Tail Surface Meshes with 2D T-Rex
# --   Refine Fuselage Surface Mesh
# --   Create Farfield
# --   Create Symmetry Plane
# --   Create Farfield Block
# --   Examine Blocks
# --   CAE Setup
# --   Save the Pointwise Project
# --------------------------------------------------------


# Start Time
set tBegin [clock seconds]
timestamp

# Load Pointwise Project File Containing Prepared OpenCSM Geometry
pw::Application load [file join $scriptDir $fileName]
pw::Display update

# Apply User Settings
pw::Connector setCalculateDimensionMethod Spacing
pw::Connector setCalculateDimensionSpacing $avgDs1

pw::DomainUnstructured setDefault BoundaryDecay $boundaryDecay
pw::DomainUnstructured setDefault Algorithm AdvancingFront

pw::Display update
pw::Application setCAESolver {NASA/FUN3D} 3

# Gather Analysis Model Information
set allDbs [pw::Database getAll]
foreach db $allDbs {
    if { [$db getDescription] == "Model"} {
        set dbModel $db

        set numQuilts [$dbModel getQuiltCount]
        for { set i 1 } { $i <= $numQuilts } { incr i } {
            lappend dbQuilts [$dbModel getQuilt $i]
        }

        # Mesh Surfaces
        if { [$dbModel isBaseForDomainUnstructured] } {
            puts "Meshing [$dbModel getName] model..."
            set surfDoms [pw::DomainUnstructured createOnDatabase -joinConnectors 80 -reject unusedSurfs $dbModel]

            if { [llength $unusedSurfs] > 0 } {
                puts "Unused surfaces exist, please check geometry."
                puts $unusedSurfs
                exit -1
            }
        } else {
            puts "Unable to mesh model."
            exit -1
        }
    }
}

# Survey Surface Mesh and Isolate Domains and Connectors
foreach qlt $dbQuilts {
    set modelDoms([$qlt getName]) [DomFromQuilt $qlt]
}

set OMLCons [ConsFromDom $modelDoms(fuselage)]
set inletIMLCons  [ConsFromDom $modelDoms(inlet-IML)]
set nozzleIMLCons [ConsFromDom $modelDoms(nozzle-IML)]

set aipCons [ConsFromDom $modelDoms(aip)]
set nozzleExitCons [ConsFromDom $modelDoms(nozzle-exit)]

set symCons          [ConsFromDom $modelDoms(symmetry)]
set tubeCons         [ConsFromDom $modelDoms(tube)]
set fuseOMLSymCons   [intersect $OMLCons $symCons]
set inletIMLSymCons  [intersect $inletIMLCons $symCons]
set nozzleIMLSymCons [intersect $nozzleIMLCons $symCons]

set freestreamCons  [ConsFromDom $modelDoms(freestream)]
set outflowCons  [ConsFromDom $modelDoms(outflow)]

## Isolate Connectors for Specific Distributions

set aipCon                [intersect $aipCons $inletIMLCons]
set nozzleExitCon         [intersect $nozzleExitCons $nozzleIMLCons]
set inletLECon            [intersect $OMLCons $inletIMLCons]
set nozzleTECon           [intersect $OMLCons $nozzleIMLCons]

set aipNode0              [$aipCon getXYZ -parameter 0.0]
set aipNode1              [$aipCon getXYZ -parameter 1.0]

set nozzleNode0           [$nozzleExitCon getXYZ -parameter 0.0]
set nozzleNode1           [$nozzleExitCon getXYZ -parameter 1.0]

## Find AIP and Nozzle Exit Plane Locations
if { [lindex $aipNode0 0] > [lindex $aipNode1 0] } {
    set aipMax $aipNode0
} else {
    set aipMax $aipNode1
}

if { [lindex $nozzleNode0 0] < [lindex $nozzleNode1 0] } {
    set nozzleMin $nozzleNode0
} else {
    set nozzleMin $nozzleNode1
}

puts "Finished isolating connectors."

# Update Display Window to More Clearly Render the Surface Mesh
pw::Display setShowDatabase 0

set allDomsCollection [pw::Collection create]
    $allDomsCollection set [pw::Grid getAll -type pw::Domain]
    $allDomsCollection do setRenderAttribute FillMode HiddenLine
$allDomsCollection delete

pw::Display update

## Adjust Domain Solver Attributes/Update Edge Spacing on Connectors

set fuseConsCollection [pw::Collection create]
    $fuseConsCollection set [lsort -unique [join [list $OMLCons            \
                                                           $inletIMLCons       \
                                                           $nozzleIMLCons      \
                                                           $aipCons            \
                                                           $nozzleExitCons]]]

   pw::Connector setCalculateDimensionSpacing $avgDs2
   $fuseConsCollection do calculateDimension
$fuseConsCollection delete

## Adjust spacings on tube, freestream and outflow connectors
foreach con $tubeCons {
    RedistCons 1.0 2 2 0.15 0.15 $con
}

foreach con $freestreamCons {
    RedistCons 1.0 2 2 0.15 0.15 $con
}

foreach con $outflowCons {
    RedistCons 1.0 2 2 0.15 0.15 $con
}

## Redistribute Internal Inlet/Nozzle Connector Distributions
foreach con $aipCons {
    RedistCons $growthRate [expr {int(log(2.0)/log($growthRate))}] [expr {int(log(2.0)/log($growthRate))}] $EFSpacing $EFSpacing $con
}

foreach con $nozzleExitCons {
    RedistCons $growthRate [expr {int(log(2.0)/log($growthRate))}] [expr {int(log(2.0)/log($growthRate))}] $EFSpacing $EFSpacing $con
}

puts "Redistributed inlet/nozzle connectors."

## Differentiate Between Inlet/Nozzle OML Connectors
foreach con $fuseOMLSymCons {

    RedistCons $inletGrowthRate [expr {int(log(12.5)/log($inletGrowthRate))}] [expr {int(log(12.5)/log($inletGrowthRate))}] $inletSpacing $inletSpacing $con
}

## Redistribute Inlet IML Connectors
foreach con $inletIMLSymCons {

    set Node0 [$con getXYZ -parameter 0.0]
    set Node1 [$con getXYZ -parameter 1.0]

    if { [lindex $Node0 0] < [lindex $Node1 0] } {
        RedistCons $inletGrowthRate [expr {int(log(12.5)/log($inletGrowthRate))}] [expr {int(log(5.0)/log($inletGrowthRate))}] $inletSpacing $EFSpacing $con
    } else {
            RedistCons $inletGrowthRate [expr {int(log(5.0)/log($inletGrowthRate))}] [expr {int(log(12.5)/log($inletGrowthRate))}] $EFSpacing $inletSpacing $con
    }
} 

## Redistribute Nozzle IML Connectors
foreach con $nozzleIMLSymCons {

    set Node0 [$con getXYZ -parameter 0.0]
    set Node1 [$con getXYZ -parameter 1.0]

    if { [lindex $Node0 0] < [lindex $Node1 0] } {
        RedistCons $nozzleGrowthRate [expr {int(log(2.5)/log($nozzleGrowthRate))}] [expr {int(log(2.5)/log($nozzleGrowthRate))}] $EFSpacing $nozzleSpacing $con
    } else {
        RedistCons $nozzleGrowthRate [expr {int(log(2.5)/log($nozzleGrowthRate))}] [expr {int(log(2.5)/log($nozzleGrowthRate))}] $nozzleSpacing $EFSpacing $con
    }
} 

pw::Display update
puts "Redistributed inlet/nozzle connectors."

## Modify Connector Distributions at Inlet/Nozzle Leading & Trailing Edges
RedistCons $leteGrowthRate [expr {int(log(5.0)/log($leteGrowthRate))}] [expr {int(log(5.05)/log($leteGrowthRate))}] $TESpacing $TESpacing [list $inletLECon $nozzleTECon]


## Resolve Inlet and Nozzle Leading/Trailing Edges with Anisotropic Triangles
set fuseDomsCollection [pw::Collection create]
    $fuseDomsCollection set [list $modelDoms(fuselage)          \
                                    $modelDoms(inlet-IML)       \
                                    $modelDoms(nozzle-IML)]
    $fuseDomsCollection do setUnstructuredSolverAttribute \
        TRexMaximumLayers $domLayers
    $fuseDomsCollection do setUnstructuredSolverAttribute \
        TRexGrowthRate $domGrowthRate

    set leBC [pw::TRexCondition create]

    $leBC setName "inlet-leading-edge"
    $leBC setType Wall
    $leBC setSpacing $inletSpacing
    $leBC apply [list \
        [list $modelDoms(fuselage) $inletLECon] \
        [list $modelDoms(inlet-IML) $inletLECon] \
    ]

    set teBC [pw::TRexCondition create]

    $teBC setName "nozzle-trailing-edge"
    $teBC setType Wall
    $teBC setSpacing $nozzleSpacing
    $teBC apply [list \
        [list $modelDoms(nozzle-IML) $nozzleTECon] \
        [list $modelDoms(fuselage) $nozzleTECon]   \
    ]

    $fuseDomsCollection do initialize
$fuseDomsCollection delete
pw::Display update


puts "Surface Meshing Complete"

## Block Assembly

set allDoms [pw::Grid getAll -type pw::Domain]
set doms {}
foreach dom $allDoms {
    lappend doms $dom
}

# Assemble Block From All Model Domains
set blks [pw::BlockUnstructured createFromDomains $doms]

set hyperloop_blk [pw::GridEntity getByName "blk-1"]
set isoMode [pw::Application begin UnstructuredSolver [list $hyperloop_blk]]
$isoMode run Initialize
$isoMode end

# Define boundary conditions

set freestreamBC [pw::BoundaryCondition create]
    $freestreamBC setName "bc-01"
    $freestreamBC apply [list $hyperloop_blk $modelDoms(freestream)]

set tubeBC [pw::BoundaryCondition create]
    $tubeBC setName "bc-02"
    $tubeBC apply [list $hyperloop_blk $modelDoms(tube)]

set symmetryBC [pw::BoundaryCondition create]
    $symmetryBC setName "bc-03"
    $symmetryBC apply [list $hyperloop_blk $modelDoms(symmetry)]

set outflowBC [pw::BoundaryCondition create]
    $outflowBC setName "bc-04"
    $outflowBC apply [list $hyperloop_blk $modelDoms(outflow)]

set aipBC [pw::BoundaryCondition create]
    $aipBC setName "bc-05"
    $aipBC apply [list $hyperloop_blk $modelDoms(aip)]

set nozzleBC [pw::BoundaryCondition create]
    $nozzleBC setName "bc-06"
    $nozzleBC apply [list $hyperloop_blk $modelDoms(nozzle-exit)]

set inletBC [pw::BoundaryCondition create]
    $inletBC setName "bc-07"
    $inletBC apply [list $hyperloop_blk $modelDoms(inlet-IML)]

set nozzleBC [pw::BoundaryCondition create]
    $nozzleBC setName "bc-08"
    $nozzleBC apply [list $hyperloop_blk $modelDoms(nozzle-IML)]
    
set fuseBC [pw::BoundaryCondition create]
    $fuseBC setName "bc-09"
    $fuseBC apply [list $hyperloop_blk $modelDoms(fuselage)]

timestamp
puts "Run Time: [convSeconds [pwu::Time elapsed $tBegin]]"

# Save the Pointwise Project
set fileRoot [file rootname $fileName]
set fileExport "$fileRoot-Grid.pw"

puts ""
puts "Writing $fileExport file..."
puts ""

pw::Application save [file join $scriptDir $fileExport]

set ioMode [pw::Application begin CaeExport [pw::Entity sort [list $hyperloop_blk $hyperloop_blk]]]
  $ioMode initialize -type CAE [file join $scriptDir "../AFLR3/Hyperloop_PW"]

  if {![$ioMode verify]} {
    error "Data verification failed."
  }
  $ioMode write
$ioMode end
unset ioMode

pw::Display update
exit

#