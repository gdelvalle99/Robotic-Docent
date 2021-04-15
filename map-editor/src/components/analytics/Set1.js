import React from "react";
import OutlinedCard from "./OutlinedCard";

export default function Set1() {
    const body = (question) => {
        return (
            <div>
                The most frequently asked question:
                <br />
                {question}
            </div>
        );
    };
    return (
        <div
            style={{
                display: "flex",
                justifyContent: "space-between",
                flexDirection: "column",
            }}
        >
            <div
                style={{
                    display: "flex",
                    justifyContent: "space-between",
                }}
            >
                <OutlinedCard
                    title="Maximum Number of Interactions"
                    fact={56}
                    subfact="Interactions at Exhibit 1"
                    // body={body("What is the statue made out of?")}
                    disableAction
                />
                <OutlinedCard
                    title="Minimum Number of Interactions"
                    fact={0}
                    subfact="There was a day where the robot did nothing"
                    // body="There was a day where the robot did nothing"
                    disableAction
                />
            </div>
            <div
                style={{
                    display: "flex",
                    justifyContent: "space-between",
                }}
            >
                <OutlinedCard
                    title="Median Number of Interactions"
                    fact={21}
                    subfact="Interactions Between All Exhibits"
                    // body={<br/>}
                    disableAction
                />
                <OutlinedCard
                    title="Average Number of Interactions"
                    fact={21.81}
                    subfact="Interactions Between All Exhibits"
                    body={body("Where is the bathroom?")}
                />
            </div>
        </div>
    );
}
