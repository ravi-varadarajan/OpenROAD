import os
import pandas as pd
import math
from sklearn.cluster import KMeans, AffinityPropagation, AgglomerativeClustering, SpectralClustering
from sklearn import preprocessing

def hvClustering(dfToCluster: pd.DataFrame):
    nodeType = dfToCluster.NodeType.values
    if nodeType[0] == "H":
        splitAt = dfToCluster.NodeLocX > dfToCluster.NodeLocX.mean()
        return splitAt.astype(int)
    splitAt = dfToCluster.NodeLocY > dfToCluster.NodeLocY.mean()
    return splitAt.astype(int)


def clusterNodes(csvFile: str, clusterSize: int = 30, clusterDiameter: float = 0.27,
        clusterAlgo: str = "KMean", mergeSmallClusters=False, outFile:str = "clusterId.txt" ):
    # Define the aggregation calculations
    aggregations = {
        # Bounds
        'NodeLocX': ['max', 'min'],
        'NodeLocY': ['max', 'min'],
        'Label': 'count'
    }
    if 'SINK_CLUSTERING_TYPE' in os.environ:
       userAlgo = os.environ['SINK_CLUSTERING_TYPE']
       if userAlgo == 'KMean' or userAlgo == 'AgglomerativeClustering' or userAlgo == 'SpectralClustering':
          clusterAlgo = userAlgo
       else:
          return
    print (f'Using clustering type: {clusterAlgo}')
    #sys.stdout.flush()

    def getBigClusters(inpDf: pd.DataFrame):
        nodeBounds = inpDf.groupby(by="Label").agg(aggregations)
        nodeBounds.columns = ["xMax", "xMin", "yMax", "yMin", "NodeCount"]
        nodeBounds['xSpan'] = nodeBounds.xMax - nodeBounds.xMin
        nodeBounds['ySpan'] = nodeBounds.yMax - nodeBounds.yMin
        bigClusters = nodeBounds[(nodeBounds.NodeCount > clusterSize) |
                                 (nodeBounds.xSpan > clusterDiameter) |
                                 (nodeBounds.ySpan > clusterDiameter)].index.values
        return bigClusters, nodeBounds
    retDf = pd.read_csv(csvFile)
    retDf["Label"] = "0"
    if clusterAlgo == "HVPartition":
        retDf["NodeType"] = "V"

    labelEncoder = preprocessing.LabelEncoder()
    nodeBounds = None
    clusterLowerBoundSize = math.ceil(clusterSize * 0.15)
    prevBadClusterNodes = []
    # print(
    #    f"Clustering Data Frame of DF of Size : {len(retDf)}, DataFrame Head : \n{retDf.head()} with algo : {clusterAlgo}")
    while True:
        bigClusters, nodeBounds = getBigClusters(retDf)
        badClusters = []
        if mergeSmallClusters:
            smallClusters = nodeBounds[(
                nodeBounds.NodeCount <= clusterLowerBoundSize)]
            badClusters = np.setdiff1d(smallClusters.index.values, bigClusters)
            if badClusters.size > 0:
                nodeIds = retDf[retDf.Label.isin(badClusters)]
                retDf.loc[nodeIds.index, "Label"] = "0"  # Reset the label
                bigClusters, _ = getBigClusters(retDf)
        if len(bigClusters) == 0 or \
                (len(prevBadClusterNodes) != 0 and np.setdiff1d(badClusters, prevBadClusterNodes).size == 0):
            break
        prevBadClusterNodes = badClusters
        for clusterId in bigClusters:
            nodeIds = retDf[retDf.Label == clusterId].index.values
            if nodeIds.size == 1:
                continue
            clusterSplit = None
            if clusterAlgo == "KMean":
                X = retDf.loc[nodeIds, ['NodeLocX', 'NodeLocY']].values
                kmeans = KMeans(n_clusters=2, random_state=0).fit(X)
                clusterSplit = kmeans.labels_
            elif clusterAlgo == "HVPartition":
                clusterSplit = hvClustering(
                    retDf.loc[nodeIds, ['NodeLocX', 'NodeLocY', 'NodeType']])
                newNodeType = "V" if retDf.loc[nodeIds,
                                               "NodeType"].values[0] == "H" else "H"
                retDf.loc[nodeIds, "NodeType"] = newNodeType
            elif clusterAlgo == "AffinityPropogation":
                X = retDf.loc[nodeIds, ['NodeLocX', 'NodeLocY']].values
                afProp = AffinityPropagation(random_state=5).fit(X)
                clusterSplit = afProp.labels_
            elif clusterAlgo == "AgglomerativeClustering":
                X = retDf.loc[nodeIds, ['NodeLocX', 'NodeLocY']].values
                agglo = AgglomerativeClustering(n_clusters=2).fit(X)
                clusterSplit = agglo.labels_
            elif clusterAlgo == "SpectralClustering":
                X = retDf.loc[nodeIds, ['NodeLocX', 'NodeLocY']].values
                spectrals = SpectralClustering(
                    n_clusters=2, assign_labels="discretize", random_state=0).fit(X)
                clusterSplit = spectrals.labels_
            retDf.loc[nodeIds, 'Label'] += clusterSplit.astype(str)

    labelEncoder.fit(retDf.Label)
    retDf['ClusterId'] = labelEncoder.transform(retDf.Label)
    retDf = retDf[['NodeLocX', 'NodeLocY', 'Label', 'ClusterId']]

    clusterDf = retDf
    boundsDf = nodeBounds
    clusterDf.sort_values(by="ClusterId")['ClusterId'].to_csv(
        "clusterId.txt", header=False, index=True, sep=" ")
    clusterDf.to_csv(
        f"{csvFile.split('.')[0]}_clusterData.csv", index=False)
    boundsDf.to_csv(
        f"{csvFile.split('.')[0]}_boundData.csv", index=False)
    return 
